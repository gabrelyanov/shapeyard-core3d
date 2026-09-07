#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <map>
#include <numeric>
#include <utility>
#include <vector>

// Private derivative only. The caller owns admission, durable metadata, exact
// corner copying and publication through the ordinary OCAF edit transaction.
namespace shapeyard::uv {
using Point = std::array<double, 3>;
using UV = std::array<double, 2>;
struct Triangle { std::array<Point, 3> points; };
struct Settings {
    int resolution = 1024;
    int gutterPixels = 4; // On every side; adjacent charts have twice this gap.
    bool valid() const noexcept {
        return (resolution == 256 || resolution == 512 || resolution == 1024
                || resolution == 2048 || resolution == 4096)
            && gutterPixels >= 1 && gutterPixels <= 32 && gutterPixels * 8 <= resolution;
    }
};
struct Atlas {
    std::vector<std::array<UV, 3>> corners;
    std::vector<int> chartForTriangle;
    int chartCount = 0;
    double occupancy = 0;
    double unitsPerMillimeter = 0;
};
namespace detail {
inline Point sub(const Point& a, const Point& b) { return {a[0]-b[0],a[1]-b[1],a[2]-b[2]}; }
inline double dot(const Point& a, const Point& b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
inline Point cross(const Point& a, const Point& b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
inline Point divide(const Point& p, double n) { return {p[0]/n,p[1]/n,p[2]/n}; }
inline double magnitude(const Point& p) { return std::sqrt(dot(p,p)); }
struct Chart {
    std::vector<int> triangles;
    Point origin, x, y, normal;
    UV low = {0,0}, high = {0,0};
    double extent = 0;
};
struct EdgeUse { int triangle; bool forward; };
using Edge = std::pair<Point, Point>;
// Strict interior intersection. Touching edges/vertices are permitted. Inputs
// are normalized by the chart extent, giving the tolerance a fixed meaning.
inline bool interiorsOverlap(const std::array<UV,3>& a, const std::array<UV,3>& b) {
    for (const auto* triangle : {&a, &b}) {
        for (int i=0;i<3;++i) {
            const auto& p=(*triangle)[i]; const auto& q=(*triangle)[(i+1)%3];
            const double nx=-(q[1]-p[1]), ny=q[0]-p[0];
            const double length=std::hypot(nx,ny);
            if (length == 0) { return true; }
            double amin=INFINITY,amax=-INFINITY,bmin=INFINITY,bmax=-INFINITY;
            for (int k=0;k<3;++k) {
                const double av=(a[k][0]*nx+a[k][1]*ny)/length;
                const double bv=(b[k][0]*nx+b[k][1]*ny)/length;
                amin=std::min(amin,av); amax=std::max(amax,av);
                bmin=std::min(bmin,bv); bmax=std::max(bmax,bv);
            }
            if (std::min(amax,bmax)-std::max(amin,bmin) <= 1.e-12) { return false; }
        }
    }
    return true;
}
}

inline bool generate(const std::vector<Triangle>& input, const Settings& settings, Atlas& output) noexcept {
    output = {};
    if (!settings.valid() || input.empty() || input.size()>4096) { return false; }
    try {
        using namespace detail;
        const int count=static_cast<int>(input.size());
        std::vector<Point> normals(count);
        std::vector<double> areas(count);
        std::map<Edge,std::vector<EdgeUse>> edges;
        for (int i=0;i<count;++i) {
            const auto& p=input[i].points;
            for (const auto& v:p) for (double c:v) if (!std::isfinite(c) || std::abs(c)>1.e6) { return false; }
            const auto e=sub(p[1],p[0]), f=sub(p[2],p[0]), n=cross(e,f);
            const double length=magnitude(e), area2=magnitude(n);
            if (!std::isfinite(area2) || length<=1.e-12 || area2<=length*length*1.e-12) { return false; }
            normals[i]=divide(n,area2); areas[i]=area2*0.5;
            for (int k=0;k<3;++k) {
                Point a=p[k],b=p[(k+1)%3];
                if (a==b) { return false; }
                const bool forward=a<b; if (!forward) std::swap(a,b);
                auto& uses=edges[{a,b}]; uses.push_back({i,forward});
                // No welding or ambiguous nonmanifold chart ownership.
                if (uses.size()>2) { return false; }
            }
        }
        std::vector<std::vector<int>> neighbors(count);
        for (const auto& entry:edges) {
            const auto& uses=entry.second;
            if (uses.size()!=2) continue;
            if (uses[0].forward==uses[1].forward) { return false; }
            const int a=uses[0].triangle,b=uses[1].triangle;
            if (dot(normals[a],normals[b])>=1.0-1.e-12) {
                neighbors[a].push_back(b); neighbors[b].push_back(a);
            }
        }
        for (auto& n:neighbors) std::sort(n.begin(),n.end());
        Atlas result; result.corners.resize(count); result.chartForTriangle.assign(count,-1);
        std::vector<Chart> charts;
        for (int seed=0;seed<count;++seed) {
            if (result.chartForTriangle[seed]>=0) continue;
            Chart chart; chart.origin=input[seed].points[0]; chart.normal=normals[seed];
            chart.x=sub(input[seed].points[1],chart.origin); chart.x=divide(chart.x,magnitude(chart.x));
            chart.y=cross(chart.normal,chart.x);
            // Membership always references the seed plane, preventing chained
            // almost-coplanar edges from gradually flattening a curved surface.
            double seedSize=0;
            for (const auto& p:input[seed].points) seedSize=std::max(seedSize,magnitude(sub(p,chart.origin)));
            const int id=static_cast<int>(charts.size());
            result.chartForTriangle[seed]=id; chart.triangles.push_back(seed);
            for (std::size_t at=0;at<chart.triangles.size();++at) {
                for (int neighbor:neighbors[chart.triangles[at]]) {
                    if (result.chartForTriangle[neighbor]>=0 || dot(chart.normal,normals[neighbor])<1.0-1.e-12) continue;
                    bool planar=true;
                    for (const auto& p:input[neighbor].points) {
                        if (std::abs(dot(sub(p,chart.origin),chart.normal))>std::max(1.e-12,seedSize*1.e-10)) planar=false;
                    }
                    if (planar) { result.chartForTriangle[neighbor]=id; chart.triangles.push_back(neighbor); }
                }
            }
            // Use a boundary edge rather than an arbitrary triangulation
            // diagonal, avoiding rotated square bounds and wasted cube space.
            double boundaryLength = 0;
            Point boundaryDirection = chart.x;
            for (int triangle : chart.triangles) for (int k=0;k<3;++k) {
                Point a=input[triangle].points[k],b=input[triangle].points[(k+1)%3];
                if (b<a) std::swap(a,b);
                const auto& uses=edges.at({a,b});
                int members=0;
                for (const auto& use:uses) if (result.chartForTriangle[use.triangle]==id) ++members;
                if (members!=1) continue;
                const auto direction=sub(b,a);const double length=magnitude(direction);
                const auto normalized=divide(direction,length);
                if (length>boundaryLength || (length==boundaryLength && normalized<boundaryDirection)) {
                    boundaryLength=length;boundaryDirection=normalized;
                }
            }
            chart.x=boundaryDirection;chart.y=cross(chart.normal,chart.x);
            chart.y=divide(chart.y,magnitude(chart.y));
            // Project the boundary direction onto the same seed plane.
            chart.x=cross(chart.y,chart.normal);
            chart.low={INFINITY,INFINITY}; chart.high={-INFINITY,-INFINITY};
            for (int i:chart.triangles) for (int k=0;k<3;++k) {
                const auto p=sub(input[i].points[k],chart.origin);
                UV uv={dot(p,chart.x),dot(p,chart.y)};
                result.corners[i][k]=uv;
                for (int axis=0;axis<2;++axis) { chart.low[axis]=std::min(chart.low[axis],uv[axis]); chart.high[axis]=std::max(chart.high[axis],uv[axis]); }
            }
            chart.extent=std::max(chart.high[0]-chart.low[0],chart.high[1]-chart.low[1]);
            if (!std::isfinite(chart.extent) || chart.extent<=0) return false;
            charts.push_back(std::move(chart));
        }
        // Bounded sweep broad phase and exact convex SAT within each chart.
        // Exhausting the work budget rejects without publishing a partial atlas.
        std::size_t comparisons=0;
        for (const auto& chart:charts) {
            struct Bounds {int triangle; double xmin,xmax,ymin,ymax;};
            std::vector<Bounds> bounds;
            for (int i:chart.triangles) {
                Bounds b{i,INFINITY,-INFINITY,INFINITY,-INFINITY};
                for (const auto& p:result.corners[i]) { b.xmin=std::min(b.xmin,p[0]);b.xmax=std::max(b.xmax,p[0]);b.ymin=std::min(b.ymin,p[1]);b.ymax=std::max(b.ymax,p[1]); }
                bounds.push_back(b);
            }
            std::sort(bounds.begin(),bounds.end(),[](const Bounds& a,const Bounds& b){return a.xmin!=b.xmin?a.xmin<b.xmin:a.triangle<b.triangle;});
            for (std::size_t i=0;i<bounds.size();++i) for (std::size_t j=i+1;j<bounds.size();++j) {
                if (++comparisons>1000000) return false;
                const auto& a=bounds[i];const auto& b=bounds[j];
                if (b.xmin>=a.xmax) break;
                if (b.ymin>=a.ymax || a.ymin>=b.ymax) continue;
                auto x=result.corners[a.triangle],y=result.corners[b.triangle];
                for (int k=0;k<3;++k) for (int d=0;d<2;++d) { x[k][d]=(x[k][d]-chart.low[d])/chart.extent;y[k][d]=(y[k][d]-chart.low[d])/chart.extent; }
                if (interiorsOverlap(x,y)) return false;
            }
        }
        std::vector<int> order(charts.size());std::iota(order.begin(),order.end(),0);
        std::sort(order.begin(),order.end(),[&](int a,int b){
            const auto& x=charts[a];const auto& y=charts[b];
            const double xh=x.high[1]-x.low[1],yh=y.high[1]-y.low[1];
            if (xh!=yh) return xh>yh;
            const double xw=x.high[0]-x.low[0],yw=y.high[0]-y.low[0];
            return xw!=yw?xw>yw:a<b;
        });
        const double gutter=double(settings.gutterPixels)/settings.resolution;
        auto pack=[&](double scale,std::vector<UV>* origins){
            double x=0,y=0,rowHeight=0;
            if (origins) origins->resize(charts.size());
            for (int i:order) {
                const auto& c=charts[i];const double w=(c.high[0]-c.low[0])*scale+2*gutter,h=(c.high[1]-c.low[1])*scale+2*gutter;
                if (w>1 || h>1) return false;
                if (x+w>1) {x=0;y+=rowHeight;rowHeight=0;}
                if (y+h>1) return false;
                if (origins) (*origins)[i]={x+gutter,y+gutter};
                x+=w;rowHeight=std::max(rowHeight,h);
            }
            return true;
        };
        double maxExtent=0;for(const auto& c:charts) maxExtent=std::max(maxExtent,c.extent);
        if (!pack(0,nullptr)) return false;
        double low=0,high=1/maxExtent;
        for(int i=0;i<48;++i){const double middle=(low+high)*0.5;if(pack(middle,nullptr))low=middle;else high=middle;}
        if (!std::isfinite(low) || low<=0) return false;
        std::vector<UV> origins;if(!pack(low,&origins))return false;
        for(int i=0;i<count;++i){
            const int id=result.chartForTriangle[i];const auto& c=charts[id];
            for(auto& uv:result.corners[i])for(int d=0;d<2;++d)uv[d]=origins[id][d]+(uv[d]-c.low[d])*low;
            // Guard flattening and floating-point collapse before publication.
            for(int k=0;k<3;++k){const int next=(k+1)%3;const auto& a=result.corners[i][k];const auto& b=result.corners[i][next];
                const double expected=magnitude(sub(input[i].points[k],input[i].points[next]))*low;
                const double actual=std::hypot(a[0]-b[0],a[1]-b[1]);
                if(!std::isfinite(actual)||actual<=1.e-14||std::abs(actual-expected)>expected*1.e-8)return false;
            }
            result.occupancy+=areas[i]*low*low;
        }
        result.unitsPerMillimeter=low;result.chartCount=static_cast<int>(charts.size());
        if(!std::isfinite(result.occupancy)||result.occupancy<=0||result.occupancy>1+1.e-10)return false;
        output=std::move(result);return true;
    } catch (...) { output={};return false; }
}
}
