#pragma once
// D1-D6 integration core. No OCCT dependency: the document supplies exact N1
// fields and LIVE face tolerance/XDirection, never a fit to triangulation nodes.
#include "ToroidalMeshUVPrototype.hpp"
#include "CurvedUVPacker.hpp"
#include <set>
#include <string>

namespace shapeyard::uv::curved {
struct FaceInput {
    int surfaceType = 3; // N1 persistent identifiers: plane/cylinder/torus/other
    int firstTriangle = 0, triangleCount = 0;
    bool reversed = false; // copy winding already incorporates this; do not flip twice
    std::vector<double> params;
    double toleranceMM = 0;
    Point xDirection{};
};
struct FaceLayout {
    curveduv::KernelTag kernel = curveduv::KernelTag::Fallback;
    int subChartCount = 0;
    double occupancy = 0, metricStretchMin = 1, metricStretchMax = 1;
    long seamCount = 0;
    std::string uvDigest; // native bridge fills SHA256 of ordered UV corner bytes
    std::string fallbackReason;
};
struct CurvedUVLayoutRecord {
    int version = 1;
    double globalTexelsPerMM = 0;
    int faceCount = 0;
    long kernelSeamEdges = 0, splitSeamEdges = 0, diagonalSeamEdges = 0, tornEdges = 0;
    std::vector<FaceLayout> faces;
    // Exact admission inputs captured at authoring. B-rep display meshes are
    // ephemeral and absent in private cold snapshots; validation replays these
    // frames/tolerances together with digest-bound persistent provenance.
    std::vector<FaceInput> regenerationFaces;
};
struct Result {
    bool ok = false;
    std::string reason;
    Atlas atlas;
    CurvedUVLayoutRecord layout;
    curveduv::PackSummary coverage;
};
namespace impl {
using namespace shapeyard::uv::detail;
struct EdgeUse { int triangle, a, b; };
using Edges = std::map<Edge, std::vector<EdgeUse>>;
inline bool edgesFor(const std::vector<Triangle>& triangles, Edges& edges) {
    for (int t=0;t<int(triangles.size());++t) for (int k=0;k<3;++k) {
        auto a=triangles[t].points[k], b=triangles[t].points[(k+1)%3];
        if (a==b) return false;
        const bool forward=a<b;
        if (!forward) std::swap(a,b);
        auto& uses=edges[{a,b}];
        uses.push_back({t,forward?k:(k+1)%3,forward?(k+1)%3:k});
        if (uses.size()>2) return false;
    }
    return true;
}
// Convert the reviewed kernels' unit-square result back into developed mm.
// Per-chart translations are irrelevant; the exact uniform kernel scale is
// removed, preserving the analytic metric (no second projection or fitting).
inline std::vector<curveduv::ChartInput> developed(const Atlas& atlas, int face,
    curveduv::KernelTag tag, std::vector<std::vector<int>>& indices, int first) {
    std::vector<curveduv::ChartInput> charts(atlas.chartCount);
    indices.resize(atlas.chartCount);
    for (auto& chart:charts) {
        chart.faceId=face; chart.kernel=tag;
        chart.rectMin={INFINITY,INFINITY}; chart.rectMax={-INFINITY,-INFINITY};
    }
    for (int t=0;t<int(atlas.corners.size());++t) {
        int id=atlas.chartForTriangle[t]; auto& chart=charts[id];
        curveduv::ChartTriangle triangle;
        for (int k=0;k<3;++k) {
            const auto& uv=atlas.corners[t][k];
            auto& p=triangle.corner[k];
            p={uv[0]/atlas.unitsPerMillimeter,uv[1]/atlas.unitsPerMillimeter};
            chart.rectMin.x=std::min(chart.rectMin.x,p.x); chart.rectMin.y=std::min(chart.rectMin.y,p.y);
            chart.rectMax.x=std::max(chart.rectMax.x,p.x); chart.rectMax.y=std::max(chart.rectMax.y,p.y);
        }
        chart.triangles.push_back(triangle); indices[id].push_back(first+t);
    }
    return charts;
}
inline bool periodicSeam(const UV& a, const UV& b, const UV& c, const UV& d,
                         double periodU, double periodV) {
    bool jumped=false;
    for (int axis=0;axis<2;++axis) {
        double x=a[axis]-c[axis], y=b[axis]-d[axis];
        double period=axis==0?periodU:periodV;
        const double tolerance=1.e-8;
        if (std::abs(x-y)>tolerance) return false; // per-endpoint branch tear
        if (std::abs(x)<=tolerance) continue;
        if (period<=0 || std::abs(std::abs(x)-period)>tolerance) return false;
        jumped=true;
    }
    return jumped;
}

}

inline Result unwrap(const std::vector<Triangle>& triangles, const std::vector<FaceInput>& faces,
                     const Settings& settings) noexcept {
    Result result;
    auto fail=[&](const std::string& reason) { Result failure; failure.reason=reason; return failure; };
    try {
        using namespace impl;
        if (!settings.valid() || settings.gutterPixels>8 || triangles.empty() || triangles.size()>4096
            || faces.empty() || faces.size()>256) return fail("invalid bounded unwrap input/settings");
        int next=0;
        for (const auto& f:faces) {
            if (f.firstTriangle!=next || f.triangleCount<=0 || f.triangleCount>int(triangles.size())-next)
                return fail("provenance triangle partition mismatch");
            next+=f.triangleCount;
        }
        if (next!=int(triangles.size())) return fail("incomplete provenance partition");
        Edges edges;
        if (!edgesFor(triangles,edges)) return fail("nonmanifold or degenerate shared edge");
        curveduv::PackerInput packer;
        packer.settings={settings.resolution,settings.gutterPixels};
        std::vector<std::vector<int>> originalIndices;
        std::vector<int> owner(triangles.size(),-1), chartOf(triangles.size(),-1), localOf(triangles.size(),-1);
        std::vector<UV> periods(faces.size(),UV{0,0});
        std::vector<std::array<UV,3>> raw(triangles.size());
        result.layout.faceCount=int(faces.size()); result.layout.faces.resize(faces.size());
        result.layout.regenerationFaces=faces;
        for (int fi=0;fi<int(faces.size());++fi) {
            const auto& f=faces[fi]; auto& record=result.layout.faces[fi];
            std::vector<Triangle> input(triangles.begin()+f.firstTriangle,triangles.begin()+f.firstTriangle+f.triangleCount);
            Atlas atlas; bool accepted=false;
            const int expected=f.surfaceType==0?9:f.surfaceType==1?10:f.surfaceType==2?11:f.surfaceType==3?0:-1;
            if (expected<0 || f.params.size()!=std::size_t(expected)) return fail("invalid analytic parameters");
            for (double p:f.params) if (!std::isfinite(p)) return fail("nonfinite analytic parameter");
            if ((f.surfaceType==1 || f.surfaceType==2) && std::isfinite(f.toleranceMM) && f.toleranceMM>0) {
                Point center{f.params[0],f.params[1],f.params[2]}, axis{f.params[3],f.params[4],f.params[5]};
                double length=magnitude(axis), xLength=magnitude(f.xDirection);
                if (length>1.e-12 && std::isfinite(xLength) && std::abs(xLength-1)<1.e-8
                    && std::abs(dot(axis,f.xDirection))<length*1.e-8) {
                    Point u,v; prototype::curved_detail::frameFor(divide(axis,length),u,v);
                    double seam=std::atan2(dot(f.xDirection,v),dot(f.xDirection,u));
                    if (f.surfaceType==1) {
                        prototype::CylinderBand surface;
                        surface.center=center; surface.axis=axis; surface.radius=f.params[9];
                        surface.seamAngle=seam; surface.fitToleranceModelUnits=f.toleranceMM;
                        const auto value=prototype::unwrapCylindricalBand(input,surface,settings);
                        accepted=value.ok(); atlas=value.atlas;
                        record.kernel=curveduv::KernelTag::Cylinder;
                        record.metricStretchMin=value.metricRelativeMin; record.metricStretchMax=value.metricRelativeMax;
                        record.fallbackReason=prototype::cylinderStatusName(value.status);
                        periods[fi]={prototype::curved_detail::kTwoPi*surface.radius,0};
                    } else {
                        prototype::TorusBand surface;
                        surface.center=center; surface.axis=axis; surface.majorRadius=f.params[9]; surface.minorRadius=f.params[10];
                        surface.seamAngleU=seam; surface.fitToleranceModelUnits=f.toleranceMM;
                        // Fixed analytic metric band, with roundoff only. The area
                        // guard includes chord error for the admitted <1.5 rad arcs.
                        double R=surface.majorRadius,r=surface.minorRadius;
                        prototype::DistortionBudgets budget{R/(R+r)-1.e-10,R/(R-r)+1.e-10,1.0};
                        const auto value=prototype::unwrapToroidalBand(input,surface,budget,settings);
                        accepted=value.ok(); atlas=value.atlas;
                        record.kernel=curveduv::KernelTag::Torus;
                        record.metricStretchMin=value.metricRelativeMin; record.metricStretchMax=value.metricRelativeMax;
                        record.fallbackReason=prototype::torusStatusName(value.status);
                        periods[fi]={prototype::curved_detail::kTwoPi*R,prototype::curved_detail::kTwoPi*r};
                    }
                } else record.fallbackReason="invalid live surface frame";
            } else record.fallbackReason="unsupported surface or missing live tolerance";
            if (accepted) {
                // The kernels anchor each triangle at corner zero. At the
                // intersection of two periodic cuts, OCCT may start the two
                // halves of a quad on opposite turns, declaring its diagonal
                // a seam. Choose the same whole-turn branch from the minimum
                // angle instead, without changing any edge vector or metric.
                Point center{f.params[0],f.params[1],f.params[2]};
                Point axis{f.params[3],f.params[4],f.params[5]}, u,v;
                axis=divide(axis,magnitude(axis));
                prototype::curved_detail::frameFor(axis,u,v);
                const auto delta=sub(input[0].points[0],center);
                const double height=dot(delta,axis);
                const auto radial=sub(delta,Point{axis[0]*height,axis[1]*height,axis[2]*height});
                const double seam=std::atan2(dot(f.xDirection,v),dot(f.xDirection,u));
                double angle[2]={std::atan2(dot(radial,v),dot(radial,u))-seam,
                    f.surfaceType==2?std::atan2(height,magnitude(radial)-f.params[9]):0};
                for(int d=0;d<2;++d) if(periods[fi][d]>0) {
                    const double turn=prototype::curved_detail::kTwoPi;
                    angle[d]=std::fmod(angle[d],turn);if(angle[d]<0)angle[d]+=turn;
                    const double period=periods[fi][d];
                    const double origin=atlas.corners[0][0][d]/atlas.unitsPerMillimeter-angle[d]*period/turn;
                    for(auto& corners:atlas.corners) {
                        double minimum=INFINITY;
                        for(const auto& uv:corners) minimum=std::min(minimum,uv[d]/atlas.unitsPerMillimeter-origin);
                        const double shift=std::floor((minimum+period*1.e-12)/period)*period;
                        for(auto& uv:corners) uv[d]-=shift*atlas.unitsPerMillimeter;
                    }
                }
                // Reject branch tears BEFORE strip splitting can disguise them as
                // split seams. Only a constant whole-period edge translation is a seam.
                for (const auto& item:edges) {
                    const auto& uses=item.second; if (uses.size()!=2) continue;
                    const auto a=uses[0],b=uses[1];
                    int ta=a.triangle-f.firstTriangle,tb=b.triangle-f.firstTriangle;
                    if (ta<0 || tb<0 || ta>=f.triangleCount || tb>=f.triangleCount) continue;
                    auto x=atlas.corners[ta],y=atlas.corners[tb];
                    for (auto* list:{&x,&y}) for (auto& uv:*list) for(double& d:uv) d/=atlas.unitsPerMillimeter;
                    bool same=true;
                    for(int d=0;d<2;++d) same=same && std::abs(x[a.a][d]-y[b.a][d])<1.e-8 && std::abs(x[a.b][d]-y[b.b][d])<1.e-8;
                    if (!same && !periodicSeam(x[a.a],x[a.b],y[b.a],y[b.b],periods[fi][0],periods[fi][1])) {
                        accepted=false; record.fallbackReason="kernel shared-edge tear"; break;
                    }
                }
            }
            if (!accepted) {
                if (!generate(input,settings,atlas)) return fail("per-face planar fallback rejected geometry");
                record.kernel=f.surfaceType==0?curveduv::KernelTag::Planar:curveduv::KernelTag::Fallback;
                record.metricStretchMin=record.metricStretchMax=1; periods[fi]={0,0};
            } else record.fallbackReason.clear();
            if (f.surfaceType==0) record.fallbackReason.clear();
            std::vector<std::vector<int>> indices;
            auto charts=developed(atlas,fi,record.kernel,indices,f.firstTriangle);
            for (int c=0;c<int(charts.size());++c) {
                int id=int(packer.charts.size());
                for(int t=0;t<int(indices[c].size());++t) {
                    int original=indices[c][t]; owner[original]=fi; chartOf[original]=id; localOf[original]=t;
                    for(int k=0;k<3;++k) raw[original][k]={charts[c].triangles[t].corner[k].x,charts[c].triangles[t].corner[k].y};
                }
                packer.charts.push_back(std::move(charts[c])); originalIndices.push_back(std::move(indices[c]));
            }
        }
        std::vector<int> flattened(triangles.size()); int offset=0;
        for (const auto& indices:originalIndices) for (int original:indices) flattened[original]=offset++;
        for (const auto& item:edges) {
            const auto& uses=item.second; if (uses.size()!=2) continue;
            const auto a=uses[0],b=uses[1]; int fi=owner[a.triangle];
            if (chartOf[a.triangle]!=chartOf[b.triangle]) continue; // explicit face/planar chart boundary, counted below
            bool seam=periodicSeam(raw[a.triangle][a.a],raw[a.triangle][a.b],raw[b.triangle][b.a],raw[b.triangle][b.b],periods[fi][0],periods[fi][1]);
            packer.adjacency.push_back({fi,flattened[a.triangle],a.a,a.b,flattened[b.triangle],b.a,b.b,seam});
        }
        const auto packed=curveduv::pack(packer);
        if (!packed.ok) return fail(packed.reason);
        result.coverage=packed.summary;
        result.atlas.corners.resize(triangles.size()); result.atlas.chartForTriangle.resize(triangles.size());
        std::vector<std::set<int>> faceCharts(faces.size());
        std::vector<std::vector<curveduv::detail::PlacedTriangle>> faceCoverage(faces.size());
        for (const auto& t:packed.triangles) {
            int original=originalIndices[t.chartIndex][t.triangleIndex];
            const int id=t.subChartIndex;
            result.atlas.chartForTriangle[original]=id; faceCharts[t.faceId].insert(id);
            curveduv::detail::PlacedTriangle coverage; coverage.subChart=id;
            for(int k=0;k<3;++k) {
                result.atlas.corners[original][k]={t.uv[k].x,t.uv[k].y};
                coverage.texel[k]={t.uv[k].x*settings.resolution,t.uv[k].y*settings.resolution};
            }
            faceCoverage[t.faceId].push_back(coverage);
        }
        // Include EVERY shared edge, including source-face and planar fallback
        // chart boundaries (the packer only accepts same-input-chart adjacency).
        for(const auto& item:edges) {
            const auto& uses=item.second; if(uses.size()!=2) continue;
            auto a=uses[0],b=uses[1];
            bool same=true; double epsilon=1.e-6/settings.resolution;
            for(int d=0;d<2;++d) same=same && std::abs(result.atlas.corners[a.triangle][a.a][d]-result.atlas.corners[b.triangle][b.a][d])<=epsilon
                && std::abs(result.atlas.corners[a.triangle][a.b][d]-result.atlas.corners[b.triangle][b.b][d])<=epsilon;
            if(chartOf[a.triangle]!=chartOf[b.triangle]) {
                if(same) ++result.coverage.seamCounts.continuous;
                else if(owner[a.triangle]!=owner[b.triangle]) {
                    // A source-face boundary is a kernel seam, never a split.
                    ++result.coverage.seamCounts.declared;
                    ++result.coverage.seamCounts.kernelSeamEdges;
                } else {
                    // A face-internal fallback chart boundary cannot certify
                    // the analytic no-diagonal-seam criterion. Keep it visible.
                    ++result.coverage.seamCounts.diagonalSeamEdges;
                    ++result.coverage.seamCounts.torn;
                }
            }
            if(!same) {
                ++result.layout.faces[owner[a.triangle]].seamCount;
                if(owner[a.triangle]!=owner[b.triangle]) ++result.layout.faces[owner[b.triangle]].seamCount;
            }
        }
        for(int f=0;f<int(faces.size());++f) {
            auto coverage=curveduv::detail::checkCoverage(faceCoverage[f],settings.resolution);
            if(!coverage.ok) return fail(coverage.reason);
            result.layout.faces[f].occupancy=double(coverage.coveredTexels)/(double(settings.resolution)*settings.resolution);
            result.layout.faces[f].subChartCount=int(faceCharts[f].size());
        }
        const auto& seams=result.coverage.seamCounts;
        result.layout.kernelSeamEdges=seams.kernelSeamEdges;
        result.layout.splitSeamEdges=seams.splitSeamEdges;
        result.layout.diagonalSeamEdges=seams.diagonalSeamEdges;
        result.layout.tornEdges=seams.torn;
        result.layout.globalTexelsPerMM=packed.summary.globalTexelsPerMM;
        result.atlas.unitsPerMillimeter=result.layout.globalTexelsPerMM/settings.resolution;
        result.atlas.chartCount=packed.summary.subChartCount; result.atlas.occupancy=packed.summary.occupancy;
        result.ok=true; return result;
    } catch(...) { return fail("curved unwrap exception"); }
}
// Shared by the native stored-atlas validator and standalone round-trip tests.
// Exact scalar comparison is deliberate: even one representable UV step fails.
inline bool validate(const std::vector<Triangle>& triangles, const std::vector<FaceInput>& faces,
                     const Settings& settings, const std::vector<std::array<UV,3>>& corners) noexcept {
    const auto regenerated=unwrap(triangles,faces,settings);
    return regenerated.ok && regenerated.atlas.corners==corners;
}
} // namespace shapeyard::uv::curved
