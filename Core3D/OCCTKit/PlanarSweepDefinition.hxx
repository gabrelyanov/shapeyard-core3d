#pragma once

// Detached numeric recipe only. This is not document, request or target authority.
// Existing closed profile codecs and their admission rules are unchanged.
#include "ProfileCurveAdmission.hxx"
#include "ProfileConstructionFrame.hxx"
#include <Precision.hxx>
#include <memory>
#include <optional>

namespace core3d::planar_sweep {
using ElementID = ProfileCurveID;
struct Definition {
    ElementID pathIdentifier = 0;
    std::vector<ProfileCurveVertex> vertices; // Open: one more vertex than edges.
    std::vector<ProfileCurveSegment> segments;
    int plane = 0; // Existing ProfilePointInPlane convention: XY, XZ, YZ.
    double radius = 0; // All lengths below are frozen native document scalars.
    double dimensionMetersPerUnit = 0;
    std::optional<profile::ConstructionFrame> constructionFrame;
};

enum class Admission {
    Accepted, InvalidCount, InvalidIdentifier, InvalidNumber, InvalidFrame,
    InvalidConnectivity, Degenerate, InvalidArc, NonTangent, TightBend,
    NonlocalContact
};
struct Inspection {
    double length = 0;          // Native document units; excludes placement scale.
    double expectedVolume = 0; // Native document units cubed; includes frame scale.
    double millimetersPerUnit = 0;
    double clearanceMM = 0;
    std::array<double, 2> firstTangent{};
};

namespace detail {
namespace analytic = profile_curve_admission;
constexpr std::size_t maximumSegments = 32;
constexpr double maximumPhysical = 1e6;
constexpr double minimumPhysical = 0.001;
constexpr double maximumNative = 1e6;
constexpr double tangentTolerance = 1e-9;
inline double edgeLength(const analytic::Edge& e) {
    return e.arc ? e.radius * std::abs(e.sweep) : analytic::distance(e.a,e.b);
}
inline analytic::Point at(const analytic::Edge& e, double fraction) {
    if (!e.arc) return analytic::add(e.a,analytic::mul(analytic::sub(e.b,e.a),fraction));
    const double a=e.start+e.sweep*fraction;
    return analytic::add(e.c,{e.radius*std::cos(a),e.radius*std::sin(a)});
}
inline analytic::Point tangent(const analytic::Edge& e, bool end) {
    if (!e.arc) {
        const auto v=analytic::sub(e.b,e.a);
        return analytic::mul(v,1/analytic::norm(v));
    }
    const double a=e.start+(end ? e.sweep : 0), sign=e.sweep>0 ? 1 : -1;
    return {-sign*std::sin(a),sign*std::cos(a)};
}
inline analytic::Edge trimmed(const analytic::Edge& e,double first,double last) {
    auto result=e;
    result.a=at(e,first); result.b=at(e,last);
    if (e.arc) { result.start=e.start+e.sweep*first;result.sweep=e.sweep*(last-first); }
    return result;
}
inline bool distant(const analytic::Edge& a,const analytic::Edge& b,double required) {
    const double d=analytic::pairDistance(a,b);
    return std::isfinite(d) && d>required;
}
}

inline Admission Inspect(const Definition& input, Inspection& output) noexcept {
    output={};
    try {
        namespace a=detail::analytic;
        if (input.segments.empty() || input.segments.size()>detail::maximumSegments
            || input.vertices.size()!=input.segments.size()+1) return Admission::InvalidCount;
        if (input.plane<0 || input.plane>2 || !std::isfinite(input.dimensionMetersPerUnit)
            || input.dimensionMetersPerUnit<=0 || input.dimensionMetersPerUnit>1e6)
            return Admission::InvalidNumber;
        const double mm=input.dimensionMetersPerUnit*1000;
        const double kernelMinimum=32*Precision::Confusion();
        const double clearance=std::max(1e-5,8*Precision::Confusion()*mm);
        const double endpointTolerance=std::max(1e-7,0.01*Precision::Confusion()*mm);
        const auto scalar=[&](double value) {
            return std::isfinite(value) && std::abs(value)<=detail::maximumNative
                && std::isfinite(value*mm) && std::abs(value*mm)<=detail::maximumPhysical;
        };
        const auto positive=[&](double value) {
            return scalar(value) && value>=kernelMinimum && value*mm>=detail::minimumPhysical;
        };
        const auto point=[&](const gp_Pnt2d& p) { return scalar(p.X()) && scalar(p.Y()); };
        if (!positive(input.radius)) return Admission::InvalidNumber;
        if (input.constructionFrame && !input.constructionFrame->IsValid()) return Admission::InvalidFrame;
        std::set<ElementID> identifiers;
        const auto claim=[&](ElementID value) { return value!=0 && identifiers.insert(value).second; };
        if (!claim(input.pathIdentifier)) return Admission::InvalidIdentifier;
        for (const auto& vertex:input.vertices) {
            if (!claim(vertex.identifier)) return Admission::InvalidIdentifier;
            if (!point(vertex.point)) return Admission::InvalidNumber;
        }
        for (const auto& segment:input.segments)
            if (!claim(segment.identifier)) return Admission::InvalidIdentifier;
        const auto origin=input.vertices.front().point;
        const auto local=[&](const gp_Pnt2d& p)->a::Point {
            // Subtract before scaling to avoid cancellation far from the origin.
            return {(p.X()-origin.X())*mm,(p.Y()-origin.Y())*mm};
        };
        std::vector<a::Edge> edges;edges.reserve(input.segments.size());
        double totalMM=0;
        for (std::size_t i=0;i<input.segments.size();++i) {
            const auto& segment=input.segments[i];
            if (segment.startVertex!=input.vertices[i].identifier
                || segment.endVertex!=input.vertices[i+1].identifier) return Admission::InvalidConnectivity;
            if (!point(segment.center) || !scalar(segment.radius)
                || !std::isfinite(segment.startDegrees) || !std::isfinite(segment.sweepDegrees))
                return Admission::InvalidNumber;
            a::Edge edge;edge.a=local(input.vertices[i].point);edge.b=local(input.vertices[i+1].point);
            edge.c=local(segment.center);edge.startID=segment.startVertex;edge.endID=segment.endVertex;
            const double chord=a::distance(edge.a,edge.b);
            if (!std::isfinite(chord) || chord<kernelMinimum*mm || chord<detail::minimumPhysical)
                return Admission::Degenerate;
            if (segment.kind==ProfileCurveKind::Line) {
                // Closed union: reject hidden arc fields rather than ignoring them.
                if (segment.center.X()!=0 || segment.center.Y()!=0 || segment.radius!=0
                    || segment.startDegrees!=0 || segment.sweepDegrees!=0) return Admission::InvalidArc;
            } else if (segment.kind==ProfileCurveKind::CircularArc) {
                if (!positive(segment.radius) || std::abs(segment.startDegrees)>360
                    || segment.sweepDegrees==0 || std::abs(segment.sweepDegrees)>180) return Admission::InvalidArc;
                edge.arc=true;edge.radius=segment.radius*mm;
                edge.start=segment.startDegrees*a::pi()/180;edge.sweep=segment.sweepDegrees*a::pi()/180;
                if (segment.radius<4*input.radius) return Admission::TightBend;
                if (a::distance(detail::at(edge,0),edge.a)>endpointTolerance
                    || a::distance(detail::at(edge,1),edge.b)>endpointTolerance) return Admission::InvalidArc;
            } else return Admission::InvalidArc;
            const double length=detail::edgeLength(edge);
            if (!std::isfinite(length) || length<kernelMinimum*mm || length<detail::minimumPhysical)
                return Admission::Degenerate;
            totalMM+=length;
            if (!std::isfinite(totalMM)) return Admission::InvalidNumber;
            if (!edges.empty()) {
                const auto previous=detail::tangent(edges.back(),true),next=detail::tangent(edge,false);
                if (a::distance(previous,next)>detail::tangentTolerance) return Admission::NonTangent;
            }
            edges.push_back(edge);
        }
        const double radiusMM=input.radius*mm,required=2*radiusMM+clearance;
        for (std::size_t i=0;i<edges.size();++i) for (std::size_t j=i+1;j<edges.size();++j) {
            if (j!=i+1) {
                // Conservative first-slice limit: no merging/resegmentation to
                // rescue valid but tightly subdivided nonadjacent segments.
                if (!detail::distant(edges[i],edges[j],required)) return Admission::NonlocalContact;
                continue;
            }
            const auto contacts=a::intersections(edges[i],edges[j]);
            if (contacts.overlap) return Admission::NonlocalContact;
            for (const auto p:contacts.points)
                if (a::distance(p,edges[i].b)>endpointTolerance) return Admission::NonlocalContact;
            // Only the G1 junction neighborhood is exempt from clearance.
            // Bend-radius and G1 admission bound this local neighborhood; the
            // detached solid must still pass the independent OCCT interference check.
            // Compare both distant tails against the entire opposite edge, so
            // adjacent long arcs cannot hide a second contact away from the join.
            const double near=3*radiusMM+clearance;
            const double left=detail::edgeLength(edges[i]),right=detail::edgeLength(edges[j]);
            if (left>near && !detail::distant(detail::trimmed(edges[i],0,1-near/left),edges[j],required))
                return Admission::NonlocalContact;
            if (right>near && !detail::distant(edges[i],detail::trimmed(edges[j],near/right,1),required))
                return Admission::NonlocalContact;
        }
        Inspection result;
        result.length=totalMM/mm;result.millimetersPerUnit=mm;result.clearanceMM=clearance;
        const auto first=detail::tangent(edges.front(),false);result.firstTangent={first.x,first.y};
        result.expectedVolume=a::pi()*input.radius*input.radius*result.length;
        if (input.constructionFrame) result.expectedVolume*=input.constructionFrame->AbsoluteVolumeScale();
        if (!std::isfinite(result.length) || !std::isfinite(result.expectedVolume) || result.expectedVolume<=0)
            return Admission::InvalidNumber;
        output=result;return Admission::Accepted;
    } catch (...) { output={};return Admission::InvalidNumber; }
}

// Freeze once before dispatch. This object contains no mutable scene handles.
class Prepared final {
public:
    const Definition definition;
    const Inspection inspection;
private:
    Prepared(Definition value,Inspection checked):definition(std::move(value)),inspection(checked) {}
    friend std::shared_ptr<const Prepared> Prepare(Definition,Admission&) noexcept;
};
inline std::shared_ptr<const Prepared> Prepare(Definition value,Admission& admission) noexcept {
    try {
        Inspection inspected;admission=Inspect(value,inspected);
        if (admission!=Admission::Accepted) return {};
        return std::shared_ptr<const Prepared>(new Prepared(std::move(value),inspected));
    } catch (...) { admission=Admission::InvalidNumber;return {}; }
}
} // namespace core3d::planar_sweep
