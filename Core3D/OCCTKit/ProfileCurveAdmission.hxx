#pragma once

// Unintegrated, uncompiled draft. Native face/solid checks still follow this
// bounded analytic admission. No flattening, document writes or kernel builds.
#include "ProfileCurveStructure.hxx"
#include <functional>
#include <utility>

namespace core3d {
namespace profile_curve_admission {
constexpr double epsilon = 1e-9;
constexpr double angleTolerance = 1e-12;
constexpr double clearance = 1e-3;
struct Point { double x = 0, y = 0; };
inline Point add(Point a, Point b) { return {a.x+b.x,a.y+b.y}; }
inline Point sub(Point a, Point b) { return {a.x-b.x,a.y-b.y}; }
inline Point mul(Point a, double k) { return {a.x*k,a.y*k}; }
inline double dot(Point a, Point b) { return a.x*b.x+a.y*b.y; }
inline double cross(Point a, Point b) { return a.x*b.y-a.y*b.x; }
inline double norm(Point a) { return std::hypot(a.x,a.y); }
inline double distance(Point a, Point b) { return norm(sub(a,b)); }
inline double angle(Point a) { return std::atan2(a.y,a.x); }
inline double pi() { return std::acos(-1.0); }
inline double modulo(double a, double period) {
    double result = std::fmod(a,period);
    return result < 0 ? result+period : result;
}
inline Point point(const gp_Pnt2d& p) { return {p.X(),p.Y()}; }
struct Edge {
    Point a,b,c;
    ProfileCurveID startID = 0, endID = 0;
    bool arc = false;
    double radius = 0, start = 0, sweep = 0;
};
using Loop = std::vector<Edge>;
inline Loop edges(const ProfileCurveLoop& loop) {
    Loop result; result.reserve(loop.segments.size());
    for (std::size_t i=0; i<loop.segments.size(); ++i) {
        const auto& s=loop.segments[i];
        Edge e;
        e.a=point(loop.vertices[i].point);
        e.b=point(loop.vertices[(i+1)%loop.vertices.size()].point);
        e.c=point(s.center); e.startID=s.startVertex; e.endID=s.endVertex;
        e.arc=s.kind==ProfileCurveKind::CircularArc;
        e.radius=s.radius; e.start=s.startDegrees*pi()/180;
        e.sweep=s.sweepDegrees*pi()/180; result.push_back(e);
    }
    return result;
}
inline bool containsAngle(const Edge& e, double t) {
    const double tau=2*pi();
    const double d=modulo(e.sweep>0 ? t-e.start : e.start-t,tau);
    return d<=std::abs(e.sweep)+angleTolerance || tau-d<angleTolerance;
}
inline double pointDistance(Point p, const Edge& e) {
    double best=std::min(distance(p,e.a),distance(p,e.b));
    if (!e.arc) {
        const Point v=sub(e.b,e.a);
        const double t=std::max(0.0,std::min(1.0,dot(sub(p,e.a),v)/dot(v,v)));
        return std::min(best,distance(p,add(e.a,mul(v,t))));
    }
    const Point d=sub(p,e.c); const double length=norm(d);
    if (length==0) return e.radius;
    return containsAngle(e,angle(d)) ? std::min(best,std::abs(length-e.radius)) : best;
}
inline std::vector<std::pair<double,double>> intervals(const Edge& e) {
    const double tau=2*pi();
    const double start=modulo(e.start+std::min(0.0,e.sweep),tau);
    const double end=start+std::abs(e.sweep);
    std::vector<std::pair<double,double>> result{{start,std::min(end,tau)}};
    if (end>tau) result.emplace_back(0,end-tau);
    return result;
}
struct Contacts { std::vector<Point> points; bool overlap=false; };
inline Contacts intersections(const Edge& a, const Edge& b) {
    if (a.arc && !b.arc) return intersections(b,a);
    Contacts result;
    if (!a.arc && !b.arc) {
        const Point v=sub(a.b,a.a),w=sub(b.b,b.a),delta=sub(b.a,a.a);
        const double den=cross(v,w);
        if (std::abs(den)>1e-14*norm(v)*norm(w)) {
            const double t=cross(delta,w)/den,s=cross(delta,v)/den;
            if (t>=-angleTolerance && t<=1+angleTolerance
                && s>=-angleTolerance && s<=1+angleTolerance)
                result.points.push_back(add(a.a,mul(v,t)));
            return result;
        }
        if (std::abs(cross(delta,v))>epsilon*norm(v)) return result;
        const double length2=dot(v,v),t=dot(delta,v)/length2;
        const double s=dot(sub(b.b,a.a),v)/length2;
        const double low=std::max(0.0,std::min(t,s)),high=std::min(1.0,std::max(t,s));
        if (high<low-angleTolerance) return result;
        result.points={add(a.a,mul(v,low)),add(a.a,mul(v,high))};
        result.overlap=(high-low)*norm(v)>epsilon;
        return result;
    }
    if (!a.arc) {
        const Point v=sub(a.b,a.a); const double length=norm(v);
        const Point u=mul(v,1/length),delta=sub(b.c,a.a);
        const double along=dot(delta,u),perp=cross(delta,u);
        const double disc=b.radius*b.radius-perp*perp;
        if (disc < -epsilon*std::max(1.0,b.radius)) return result;
        const double half=std::sqrt(std::max(0.0,disc));
        for (double t : {along-half,along+half}) {
            const Point p=add(a.a,mul(u,t));
            if (t>=-epsilon && t<=length+epsilon && containsAngle(b,angle(sub(p,b.c))))
                result.points.push_back(p);
        }
        return result;
    }
    const Point delta=sub(b.c,a.c); const double d=norm(delta);
    if (d<epsilon) {
        if (std::abs(a.radius-b.radius)>epsilon) return result;
        for (const auto& ai:intervals(a)) for (const auto& bi:intervals(b))
            if (std::min(ai.second,bi.second)-std::max(ai.first,bi.first)>angleTolerance)
                result.overlap=true;
        for (Point p : {a.a,a.b,b.a,b.b})
            if (containsAngle(a,angle(sub(p,a.c))) && containsAngle(b,angle(sub(p,b.c))))
                result.points.push_back(p);
        return result;
    }
    if (d>a.radius+b.radius+epsilon || d<std::abs(a.radius-b.radius)-epsilon) return result;
    const double along=(a.radius*a.radius-b.radius*b.radius+d*d)/(2*d);
    const double height2=a.radius*a.radius-along*along;
    if (height2 < -epsilon*std::max(1.0,a.radius)) return result;
    const Point u=mul(delta,1/d),perp{-u.y,u.x},base=add(a.c,mul(u,along));
    const double height=std::sqrt(std::max(0.0,height2));
    for (double sign : {-1.0,1.0}) {
        const Point p=add(base,mul(perp,sign*height));
        if (containsAngle(a,angle(sub(p,a.c))) && containsAngle(b,angle(sub(p,b.c))))
            result.points.push_back(p);
    }
    return result;
}
inline double pairDistance(const Edge& a, const Edge& b) {
    const auto contacts=intersections(a,b);
    if (contacts.overlap || !contacts.points.empty()) return 0;
    double best=std::min({pointDistance(a.a,b),pointDistance(a.b,b),
                          pointDistance(b.a,a),pointDistance(b.b,a)});
    if (a.arc && !b.arc) return pairDistance(b,a);
    if (!a.arc && b.arc) {
        const Point v=sub(a.b,a.a); const double length=norm(v);
        const Point u=mul(v,1/length),perp{-u.y,u.x};
        const double t=dot(sub(b.c,a.a),u);
        if (t>=0 && t<=length) {
            const Point foot=add(a.a,mul(u,t));
            for (double sign : {-1.0,1.0}) {
                const Point p=add(b.c,mul(perp,sign*b.radius));
                if (containsAngle(b,angle(sub(p,b.c)))) best=std::min(best,distance(foot,p));
            }
        }
    } else if (a.arc && b.arc) {
        const Point delta=sub(b.c,a.c); const double d=norm(delta);
        if (d>epsilon) {
            const Point u=mul(delta,1/d);
            for (double sa : {-1.0,1.0}) for (double sb : {-1.0,1.0}) {
                const Point p=add(a.c,mul(u,sa*a.radius)),q=add(b.c,mul(u,sb*b.radius));
                if (containsAngle(a,angle(sub(p,a.c))) && containsAngle(b,angle(sub(q,b.c))))
                    best=std::min(best,distance(p,q));
            }
        }
    }
    return best;
}
// Caller has established positive separation from the boundary. Choose a ray
// between endpoint/tangent directions instead of perturbing geometry or inputs.
inline bool inside(Point p, const Loop& loop) {
    std::vector<double> bad; bad.reserve(loop.size()*4);
    for (const auto& e:loop) {
        for (Point endpoint : {e.a,e.b}) bad.push_back(modulo(angle(sub(endpoint,p)),pi()));
        if (e.arc) {
            const Point delta=sub(e.c,p); const double d=norm(delta);
            if (d>=e.radius) {
                const double offset=std::asin(std::min(1.0,e.radius/d)),t=angle(delta);
                bad.push_back(modulo(t-offset,pi()));bad.push_back(modulo(t+offset,pi()));
            }
        }
    }
    std::sort(bad.begin(),bad.end());
    double gap=-1,start=0;
    for (std::size_t i=0;i<bad.size();++i) {
        const double next=i+1<bad.size()?bad[i+1]:bad.front()+pi();
        const double candidate=next-bad[i];
        if (candidate>gap || (candidate==gap && bad[i]>start)) { gap=candidate;start=bad[i]; }
    }
    const double theta=start+gap/2;
    const Point direction{std::cos(theta),std::sin(theta)},perp{-direction.y,direction.x};
    std::size_t hits=0;
    for (const auto& e:loop) {
        if (!e.arc) {
            const Point a=sub(e.a,p),b=sub(e.b,p);const double ay=dot(a,perp),by=dot(b,perp);
            if ((ay>0)!=(by>0)) {
                const double t=-ay/(by-ay);
                if (dot(add(a,mul(sub(b,a),t)),direction)>0) ++hits;
            }
        } else {
            const Point c=sub(e.c,p); const double cx=dot(c,direction),cy=dot(c,perp);
            if (std::abs(cy)>=e.radius) continue;
            const double half=std::sqrt(e.radius*e.radius-cy*cy);
            for (double x : {cx-half,cx+half}) {
                const Point hit=add(p,mul(direction,x));
                if (x>0 && containsAngle(e,angle(sub(hit,e.c)))) ++hits;
            }
        }
    }
    return hits%2==1;
}
} // namespace profile_curve_admission

// Shared by immutable request admission and persistence decoding. The worker
// must additionally build and check the exact OCCT face/solid before publishing.
struct ProfileCurveSectionInspection {
    double area = 0;
    double firstMomentX = 0; // Absolute authored U coordinate, independent of winding.
    std::array<double,4> outerBounds{};
    std::vector<std::array<double,4>> innerBounds;
};
inline bool InspectProfileCurveSection(const ProfileCurveSection& section,
    ProfileCurveSectionInspection& output,
    const std::function<bool()>& cancelled = {}) noexcept {
    output = {};
    try {
        using namespace profile_curve_admission;
        if (section.inner.size()>16 || section.outer.vertices.empty()) return false;
        std::set<ProfileCurveID> ids;std::size_t vertexCount=0,segmentCount=0;
        const gp_Pnt2d origin=section.outer.vertices.front().point;
        ProfileCurveLoopInspection inspection;
        ProfileCurveSectionInspection measured;
        double relativeMoment = 0;
        std::vector<Loop> loops;loops.reserve(section.inner.size()+1);
        const auto admitStructure=[&](const ProfileCurveLoop& loop) {
            if (!InspectProfileCurveLoopStructure(loop,origin,ids,vertexCount,segmentCount,inspection))
                return false;
            const double role=loops.empty()?1.0:-1.0;
            const double orientation=inspection.signedArea>0?1.0:-1.0;
            if (loops.empty()) measured.outerBounds=inspection.bounds;
            else measured.innerBounds.push_back(inspection.bounds);
            measured.area+=role*std::abs(inspection.signedArea);
            relativeMoment+=role*orientation*inspection.signedFirstMomentX;
            loops.push_back(edges(loop));return true;
        };
        if (!admitStructure(section.outer)) return false;
        for (const auto& loop:section.inner) if (!admitStructure(loop)) return false;
        for (const auto& loop:loops) for (std::size_t i=0;i<loop.size();++i) {
            if (cancelled && cancelled()) return false;
            for (std::size_t j=i+1;j<loop.size();++j) {
                const auto& a=loop[i];const auto& b=loop[j];
                const auto contacts=intersections(a,b);
                if (contacts.overlap) return false;
                std::vector<Point> allowed;
                if (a.startID==b.startID || a.startID==b.endID) allowed.push_back(a.a);
                if (a.endID==b.startID || a.endID==b.endID) allowed.push_back(a.b);
                if (!allowed.empty()) {
                    for (Point p:contacts.points) {
                        double nearest=std::numeric_limits<double>::infinity();
                        for (Point q:allowed) nearest=std::min(nearest,distance(p,q));
                        if (nearest>1e-7) return false;
                    }
                } else if (pairDistance(a,b)<clearance) return false;
            }
        }
        for (std::size_t i=1;i<loops.size();++i) {
            for (std::size_t j=0;j<i;++j) for (const auto& a:loops[i]) {
                if (cancelled && cancelled()) return false;
                for (const auto& b:loops[j]) if (pairDistance(a,b)<clearance) return false;
            }
            if (!inside(loops[i].front().a,loops.front())) return false;
            for (std::size_t j=1;j<i;++j)
                if (inside(loops[i].front().a,loops[j]) || inside(loops[j].front().a,loops[i]))
                    return false;
        }
        measured.firstMomentX=relativeMoment+origin.X()*measured.area;
        if ((cancelled && cancelled()) || !std::isfinite(measured.area) || measured.area<1e-6
            || !std::isfinite(measured.firstMomentX)) return false;
        output=measured;return true;
    } catch (...) { output={};return false; }
}
inline bool ValidateProfileCurveSection(const ProfileCurveSection& section,
    const std::function<bool()>& cancelled = {}) noexcept {
    ProfileCurveSectionInspection inspection;
    return InspectProfileCurveSection(section,inspection,cancelled);
}
} // namespace core3d
