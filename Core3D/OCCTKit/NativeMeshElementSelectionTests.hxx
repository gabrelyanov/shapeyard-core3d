#pragma once
// DEBUG selection-policy checks, executed by native XCTest through the guarded runner.
#include "NativeMeshElementSelection.hpp"
#include <limits>
#include <optional>
#include <string>

#if DEBUG
namespace core3d::meshedit {
inline std::map<std::string, bool> DebugElementSelectionPolicy() {
    std::map<std::string, bool> checks;
    std::atomic_bool cancelled{false};
    const Point a{0,0,0}, b{10,0,0}, c{10,10,0}, d{0,10,0};
    Topology square;
    checks["squareAnalyzed"] = Analyze({Triangle{a,b,c}, Triangle{a,c,d}},
                                       square, cancelled) == TopologyResult::Ready;
    if (!checks["squareAnalyzed"]) return checks;
    std::vector<std::uint32_t> output;
    auto resolvedPoints = [&]() {
        std::set<Point> points;
        for (const auto id : output) points.insert(square.vertices.at(id).point);
        return points;
    };
    checks["adjacentTrianglesShareVertices"] =
        ResolveElementVertices(square, ElementKind::Triangle, {0,1}, output, cancelled)
            == ElementSelectionResult::Ready && output.size() == 4
        && resolvedPoints() == std::set<Point>{a,b,c,d};
    std::optional<std::uint32_t> ab, bc, diagonal;
    for (std::size_t i=0; i<square.edges.size(); ++i) {
        const auto& edge = square.edges[i];
        const std::set<Point> points{square.vertices.at(edge.vertices[0]).point,
                                    square.vertices.at(edge.vertices[1]).point};
        if (points == std::set<Point>{a,b}) ab = static_cast<std::uint32_t>(i);
        if (points == std::set<Point>{b,c}) bc = static_cast<std::uint32_t>(i);
        if (edge.uses.size() == 2) diagonal = static_cast<std::uint32_t>(i);
    }
    checks["twoEdgesShareEndpoint"] = ab && bc
        && ResolveElementVertices(square, ElementKind::Edge, {*ab,*bc}, output, cancelled)
            == ElementSelectionResult::Ready && output.size() == 3
        && resolvedPoints() == std::set<Point>{a,b,c};
    checks["interiorEdgeUsesTwoVertices"] = diagonal
        && ResolveElementVertices(square, ElementKind::Edge, {*diagonal}, output, cancelled)
            == ElementSelectionResult::Ready && output.size() == 2
        && resolvedPoints() == std::set<Point>{a,c};
    auto rejects = [&](ElementKind kind, const std::vector<std::uint32_t>& ids,
                       ElementSelectionResult expected) {
        output = {999};
        const auto result = ResolveElementVertices(square, kind, ids, output, cancelled);
        return result == expected && output.empty();
    };
    checks["duplicateElementRejected"] = rejects(ElementKind::Triangle, {0,0}, ElementSelectionResult::Invalid);
    checks["invalidIDRejected"] = rejects(ElementKind::Edge,
        {std::numeric_limits<std::uint32_t>::max()}, ElementSelectionResult::Invalid);
    checks["unknownKindRejected"] = rejects(static_cast<ElementKind>(255), {0}, ElementSelectionResult::Invalid);
    checks["emptyRejected"] = rejects(ElementKind::Vertex, {}, ElementSelectionResult::Invalid);
    cancelled.store(true);
    checks["cancelledOutputCleared"] = rejects(ElementKind::Triangle, {0}, ElementSelectionResult::Cancelled);
    cancelled.store(false);

    // Twenty-two separate triangles: no coincident positions or shared fan.
    // Element count22 is allowed, but its66 unique moved vertices are not.
    std::vector<Triangle> input;
    for (int i=0; i<22; ++i) {
        const double x=3.0*i;
        input.push_back(Triangle{Point{x,0,0}, Point{x+1,0,0}, Point{x,1,0}});
    }
    Topology many;
    checks["manyAnalyzed"] = Analyze(input, many, cancelled) == TopologyResult::Ready;
    if (!checks["manyAnalyzed"]) return checks;
    std::vector<std::uint32_t> selected;
    for (std::uint32_t i=0; i<21; ++i) selected.push_back(i);
    checks["sixtyThreeVerticesAccepted"] =
        ResolveElementVertices(many, ElementKind::Triangle, selected, output, cancelled)
            == ElementSelectionResult::Ready && output.size() == 63;
    selected.push_back(21);
    checks["expandedVertexBudgetEnforced"] =
        ResolveElementVertices(many, ElementKind::Triangle, selected, output, cancelled)
            == ElementSelectionResult::TooLarge && output.empty();
    selected.clear();
    for (std::uint32_t i=0; i<64; ++i) selected.push_back(i);
    checks["sixtyFourVerticesAccepted"] =
        ResolveElementVertices(many, ElementKind::Vertex, selected, output, cancelled)
            == ElementSelectionResult::Ready && output.size() == 64;
    selected.push_back(64);
    checks["elementBudgetEnforced"] =
        ResolveElementVertices(many, ElementKind::Vertex, selected, output, cancelled)
            == ElementSelectionResult::TooLarge && output.empty();
    return checks;
}
} // namespace core3d::meshedit
#endif
