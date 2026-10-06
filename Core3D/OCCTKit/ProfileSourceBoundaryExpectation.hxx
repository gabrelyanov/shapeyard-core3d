#pragma once

// Values-only expected boundary for a complete canonical Profile extrusion.
// Candidate geometry, native history and document state are deliberately absent.
#include "ProfilePersistence.hxx"
#include <gp_Pnt.hxx>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <set>
#include <utility>
#include <vector>

namespace core3d::complete_profile_source {

enum class LoopRepresentation {
    ExplicitSegments,
    Polygon,
    PeriodicCircle
};
enum class SegmentSupport {
    Line,
    CircularArc,
    PeriodicCircle
};

// Ordinals are always present. Authored IDs are present only for explicit
// curve loops; legacy polygon/circle recipes have no IDs to invent.
struct SemanticKey {
    std::size_t loopOrdinal = 0;
    std::size_t elementOrdinal = 0;
    std::optional<ProfileCurveID> loopIdentifier;
    std::optional<ProfileCurveID> elementIdentifier;
};

struct AngularInterval {
    // Preserve the authored, unwrapped interval. In particular, endRadians is
    // not reduced modulo 2*pi and a negative sweep remains negative.
    double startDegrees = 0;
    double sweepDegrees = 0;
    double startRadians = 0;
    double endRadians = 0;
};

struct ExpectedVertexPair {
    SemanticKey key;
    std::optional<ProfileCurveID> authoredIdentifier;
    gp_Pnt lower;
    gp_Pnt upper;
};

struct ExpectedBoundaryEdgePair {
    SemanticKey key;
    std::optional<ProfileCurveID> authoredIdentifier;
    std::size_t startVertex = 0;
    std::size_t endVertex = 0;
    SegmentSupport support = SegmentSupport::Line;
    std::optional<AngularInterval> interval;
};

struct ExpectedExtrusionEdge {
    SemanticKey key;
    std::size_t vertex = 0;
    // True only for an implicit circle's recipe-derived zero-angle seam.
    bool periodicSeamOwner = false;
};

enum class EdgeLevel { LowerBoundary, UpperBoundary, Extrusion };
struct ExpectedEdgeUse {
    EdgeLevel level = EdgeLevel::LowerBoundary;
    std::size_t edge = 0;
    bool forward = false;
};

struct ExpectedLoop {
    SemanticKey key;
    LoopRepresentation representation = LoopRepresentation::Polygon;
    bool inner = false;
    int authoredWinding = 1;
    int materialParity = 1;
    int chartParity = 1;
    int frameParity = 1;
    std::vector<std::size_t> vertices;
    std::vector<std::size_t> boundaryEdges;
    std::optional<std::size_t> periodicSeamEdge;
};

struct ExpectedCapWire {
    std::size_t loop = 0;
    // Product of authored winding, outer/inner material role, work-plane chart,
    // source-frame handedness and lower/upper outward direction.
    int outwardUseParity = 1;
    std::vector<ExpectedEdgeUse> uses;
};
struct ExpectedCapFace {
    bool upper = false;
    std::vector<ExpectedCapWire> wires;
};

struct ExpectedSideFace {
    SemanticKey key;
    std::size_t loop = 0;
    std::size_t boundaryEdge = 0;
    SegmentSupport support = SegmentSupport::Line;
    int outwardUseParity = 1;
    std::optional<AngularInterval> interval;
    // A periodic circle owns one explicit seam edge twice, with opposite uses.
    // An explicit multi-arc loop never acquires such a seam.
    std::optional<std::size_t> periodicSeamOwner;
    std::array<ExpectedEdgeUse, 4> boundary{};
};

struct ExtrusionExpectation {
    // Exact scalar order emitted by profile::Encode for this complete P value.
    std::vector<double> canonicalValues;
    double metersPerUnit = 0;
    double millimetresPerUnit = 0;
    bool constructionFramePresent = false;
    std::array<double, 8> constructionFrameValues{0, 0, 0, 0, 0, 0, 1, 1};
    int plane = 0;
    double depth = 0;
    std::vector<ExpectedVertexPair> vertices;
    std::vector<ExpectedBoundaryEdgePair> boundaryEdges;
    std::vector<ExpectedExtrusionEdge> extrusionEdges;
    std::vector<ExpectedLoop> loops;
    std::array<ExpectedCapFace, 2> caps;
    std::vector<ExpectedSideFace> sides;

    std::size_t vertexCellCount() const noexcept { return 2 * vertices.size(); }
    std::size_t edgeCellCount() const noexcept {
        return 2 * boundaryEdges.size() + extrusionEdges.size();
    }
    std::size_t faceCellCount() const noexcept { return 2 + sides.size(); }
};

namespace detail {
inline gp_Pnt PointAt(const profile::Parameters& parameters,
                      const gp_Pnt2d& point, double height) {
    gp_Pnt result;
    switch (parameters.definition.plane) {
        case 0: result = gp_Pnt(point.X(), point.Y(), height); break;
        case 1: result = gp_Pnt(point.X(), height, point.Y()); break;
        default: result = gp_Pnt(height, point.X(), point.Y()); break;
    }
    if (parameters.constructionFrame) {
        gp_Trsf transform;
        if (!parameters.constructionFrame->Transform(transform)) return {};
        result.Transform(transform);
    }
    return result;
}

struct LoopValues {
    LoopRepresentation representation = LoopRepresentation::Polygon;
    std::optional<ProfileCurveID> identifier;
    std::vector<ProfileCurveVertex> vertices;
    std::vector<ProfileCurveSegment> segments;
    int authoredWinding = 1;
};

inline ProfileCurveSegment LineSegment(std::size_t index,
                                       const std::vector<ProfileCurveVertex>& vertices) {
    ProfileCurveSegment segment;
    segment.startVertex = vertices[index].identifier;
    segment.endVertex = vertices[(index + 1) % vertices.size()].identifier;
    return segment;
}

inline LoopValues PolygonLoop(const std::vector<gp_Pnt2d>& points) {
    LoopValues result;
    result.vertices.reserve(points.size()); result.segments.reserve(points.size());
    double signedArea = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        ProfileCurveVertex vertex;
        vertex.point = points[i];
        result.vertices.push_back(vertex);
        signedArea += ProfileCross(points.front(), points[i], points[(i + 1) % points.size()]) * .5;
    }
    for (std::size_t i = 0; i < points.size(); ++i)
        result.segments.push_back(LineSegment(i, result.vertices));
    result.authoredWinding = signedArea < 0 ? -1 : 1;
    return result;
}

inline LoopValues CircleLoop(const gp_Pnt2d& center, double radius) {
    LoopValues result;
    result.representation = LoopRepresentation::PeriodicCircle;
    ProfileCurveVertex seam;
    seam.point = gp_Pnt2d(center.X() + radius, center.Y());
    result.vertices.push_back(seam);
    ProfileCurveSegment circle;
    circle.startVertex = 0; circle.endVertex = 0;
    circle.kind = ProfileCurveKind::CircularArc;
    circle.center = center; circle.radius = radius;
    circle.startDegrees = 0; circle.sweepDegrees = 360;
    result.segments.push_back(circle);
    return result;
}

inline bool ExplicitLoop(const ProfileCurveLoop& input, LoopValues& output,
                         std::set<ProfileCurveID>& identifiers,
                         std::size_t& vertexCount, std::size_t& segmentCount,
                         ProfileProducerAccounting& accounting) {
    ProfileCurveLoopInspection inspection;
    if (!InspectProfileCurveLoopStructure(input, input.vertices.front().point,
            identifiers, vertexCount, segmentCount, inspection, accounting)) return false;
    output = {};
    output.representation = LoopRepresentation::ExplicitSegments;
    output.identifier = input.identifier;
    output.vertices = input.vertices;
    output.segments = input.segments;
    output.authoredWinding = inspection.signedArea < 0 ? -1 : 1;
    return true;
}
} // namespace detail

inline bool ConstructExtrusionExpectation(const profile::Parameters& parameters,
                                           ProfileProducerAccounting& accounting,
                                           ExtrusionExpectation& output) noexcept {
    output = {};
    try {
        const auto& definition = parameters.definition;
        ExtrusionExpectation result;
        // Shell transitions and revolutions have separate constructors. This
        // boundary never drops either feature to recover an extrusion match.
        if (accounting.stopped() || definition.revolve || !parameters.shells.empty()
            || !profile::Encode(parameters, result.canonicalValues)
            || !accounting.visit(result.canonicalValues.size())) return false;
        result.metersPerUnit = parameters.metersPerUnit;
        result.millimetresPerUnit = 1000.0 * parameters.metersPerUnit;
        if (!std::isfinite(result.millimetresPerUnit) || result.millimetresPerUnit <= 0)
            return false;
        result.constructionFramePresent = parameters.constructionFrame.has_value();
        if (parameters.constructionFrame)
            result.constructionFrameValues = parameters.constructionFrame->values;
        result.plane = definition.plane; result.depth = definition.depth;
        const int chartParity = definition.plane == 1 ? -1 : 1;
        const int frameParity = parameters.constructionFrame
            && parameters.constructionFrame->values[7] < 0 ? -1 : 1;

        std::vector<detail::LoopValues> sourceLoops;
        if (definition.curves) {
            std::set<ProfileCurveID> identifiers;
            std::size_t vertexCount = 0, segmentCount = 0;
            detail::LoopValues loop;
            if (!detail::ExplicitLoop(definition.curves->outer, loop, identifiers,
                    vertexCount, segmentCount, accounting)) return false;
            sourceLoops.push_back(std::move(loop));
            for (const auto& inner : definition.curves->inner) {
                if (!detail::ExplicitLoop(inner, loop, identifiers,
                        vertexCount, segmentCount, accounting)) return false;
                sourceLoops.push_back(std::move(loop));
            }
        } else if (definition.circle) {
            sourceLoops.push_back(detail::CircleLoop(definition.circle->center,
                                                      definition.circle->outerRadius));
            if (definition.circle->innerRadius > 0)
                sourceLoops.push_back(detail::CircleLoop(definition.circle->center,
                                                          definition.circle->innerRadius));
        } else {
            sourceLoops.push_back(detail::PolygonLoop(definition.points));
            for (const auto& hole : definition.holes)
                sourceLoops.push_back(detail::CircleLoop(hole.center, hole.radius));
        }
        if (sourceLoops.empty()) return false;
        // Encode/structural admission above traverses canonical values and every
        // explicit ID. Reserve the remaining values-only passes independently:
        // loop materialization, all containment pairs, and the expected cells,
        // transforms and oriented uses built below. Bounds are inherited from
        // Profile admission (17 loops / 512 vertices and segments).
        if (!accounting.visit(sourceLoops.size())) return false;
        const std::size_t containmentPairs =
            sourceLoops.size() * (sourceLoops.size() - 1) / 2;
        if (!accounting.visit(containmentPairs)) return false;

        for (std::size_t loopIndex = 0; loopIndex < sourceLoops.size(); ++loopIndex) {
            const auto& source = sourceLoops[loopIndex];
            if (source.vertices.empty() || source.segments.size() != source.vertices.size()) return false;
            ExpectedLoop loop;
            loop.key.loopOrdinal = loopIndex;
            loop.key.loopIdentifier = source.identifier;
            loop.representation = source.representation;
            loop.inner = loopIndex != 0;
            loop.authoredWinding = source.authoredWinding;
            loop.materialParity = loop.inner ? -1 : 1;
            loop.chartParity = chartParity; loop.frameParity = frameParity;
            const std::size_t firstVertex = result.vertices.size();
            for (std::size_t i = 0; i < source.vertices.size(); ++i) {
                // Semantic key plus lower/upper plane-then-frame transforms.
                if (!accounting.visit(3)) return false;
                ExpectedVertexPair vertex;
                vertex.key = {loopIndex, i, source.identifier,
                    source.vertices[i].identifier == 0
                        ? std::optional<ProfileCurveID>{}
                        : std::optional<ProfileCurveID>{source.vertices[i].identifier}};
                vertex.authoredIdentifier = vertex.key.elementIdentifier;
                vertex.lower = detail::PointAt(parameters, source.vertices[i].point, 0);
                vertex.upper = detail::PointAt(parameters, source.vertices[i].point, definition.depth);
                loop.vertices.push_back(result.vertices.size());
                result.vertices.push_back(std::move(vertex));
            }
            const std::size_t firstBoundary = result.boundaryEdges.size();
            for (std::size_t i = 0; i < source.segments.size(); ++i) {
                // Segment/ID traversal and its complete authored interval.
                if (!accounting.visit(2)) return false;
                const auto& segment = source.segments[i];
                ExpectedBoundaryEdgePair edge;
                edge.key = {loopIndex, i, source.identifier,
                    segment.identifier == 0 ? std::optional<ProfileCurveID>{}
                                            : std::optional<ProfileCurveID>{segment.identifier}};
                edge.authoredIdentifier = edge.key.elementIdentifier;
                edge.startVertex = firstVertex + i;
                edge.endVertex = firstVertex + (i + 1) % source.vertices.size();
                if (source.representation == LoopRepresentation::PeriodicCircle) {
                    edge.support = SegmentSupport::PeriodicCircle;
                    edge.endVertex = edge.startVertex;
                    edge.interval = AngularInterval{0, 360, 0, 2 * std::acos(-1.0)};
                } else if (segment.kind == ProfileCurveKind::CircularArc) {
                    edge.support = SegmentSupport::CircularArc;
                    const double first = segment.startDegrees * std::acos(-1.0) / 180.0;
                    edge.interval = AngularInterval{segment.startDegrees, segment.sweepDegrees,
                        first, first + segment.sweepDegrees * std::acos(-1.0) / 180.0};
                } else if (segment.kind != ProfileCurveKind::Line) return false;
                loop.boundaryEdges.push_back(result.boundaryEdges.size());
                result.boundaryEdges.push_back(std::move(edge));
            }
            for (std::size_t i = 0; i < source.vertices.size(); ++i) {
                if (!accounting.visit(1)) return false;
                ExpectedExtrusionEdge edge;
                edge.key = {loopIndex, i, source.identifier,
                    source.vertices[i].identifier == 0
                        ? std::optional<ProfileCurveID>{}
                        : std::optional<ProfileCurveID>{source.vertices[i].identifier}};
                edge.vertex = firstVertex + i;
                edge.periodicSeamOwner = source.representation == LoopRepresentation::PeriodicCircle;
                const std::size_t index = result.extrusionEdges.size();
                result.extrusionEdges.push_back(std::move(edge));
                if (source.representation == LoopRepresentation::PeriodicCircle)
                    loop.periodicSeamEdge = index;
            }
            result.loops.push_back(std::move(loop));
            for (std::size_t i = 0; i < source.segments.size(); ++i) {
                // One expected side cell and its four oriented boundary uses.
                if (!accounting.visit(5)) return false;
                ExpectedSideFace side;
                side.key = result.boundaryEdges[firstBoundary + i].key;
                side.loop = loopIndex; side.boundaryEdge = firstBoundary + i;
                side.support = result.boundaryEdges[firstBoundary + i].support;
                side.interval = result.boundaryEdges[firstBoundary + i].interval;
                side.outwardUseParity = result.loops.back().authoredWinding
                    * result.loops.back().materialParity * chartParity * frameParity;
                const std::size_t start = firstVertex + i;
                const std::size_t end = firstVertex + (i + 1) % source.vertices.size();
                const std::size_t startWall = start;
                const std::size_t endWall = end;
                side.boundary = {{{EdgeLevel::LowerBoundary, firstBoundary + i, true},
                                  {EdgeLevel::Extrusion, endWall, true},
                                  {EdgeLevel::UpperBoundary, firstBoundary + i, false},
                                  {EdgeLevel::Extrusion, startWall, false}}};
                if (source.representation == LoopRepresentation::PeriodicCircle) {
                    side.periodicSeamOwner = result.loops.back().periodicSeamEdge;
                    side.boundary[1].edge = *side.periodicSeamOwner;
                    side.boundary[3].edge = *side.periodicSeamOwner;
                }
                result.sides.push_back(std::move(side));
            }
        }

        for (unsigned capIndex = 0; capIndex < 2; ++capIndex) {
            auto& cap = result.caps[capIndex]; cap.upper = capIndex == 1;
            for (std::size_t loopIndex = 0; loopIndex < result.loops.size(); ++loopIndex) {
                const auto& loop = result.loops[loopIndex]; ExpectedCapWire wire;
                if (!accounting.visit(1 + loop.boundaryEdges.size())) return false;
                wire.loop = loopIndex;
                wire.outwardUseParity = loop.authoredWinding * loop.materialParity
                    * loop.chartParity * loop.frameParity * (cap.upper ? 1 : -1);
                for (auto edge : loop.boundaryEdges)
                    wire.uses.push_back({cap.upper ? EdgeLevel::UpperBoundary
                                                   : EdgeLevel::LowerBoundary,
                                         edge, wire.outwardUseParity > 0});
                cap.wires.push_back(std::move(wire));
            }
        }
        if (result.vertexCellCount() != 2 * result.vertices.size()
            || result.edgeCellCount() != 2 * result.boundaryEdges.size()
                                          + result.extrusionEdges.size()
            || result.faceCellCount() != result.sides.size() + 2) return false;
        output = std::move(result); return true;
    } catch (...) { output = {}; return false; }
}

} // namespace core3d::complete_profile_source
