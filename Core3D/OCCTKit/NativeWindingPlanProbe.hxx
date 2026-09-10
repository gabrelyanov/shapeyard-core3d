#pragma once
// DEBUG probe exercised through guarded XCTest only.
#if DEBUG
#include "NativeMeshWindingPlan.hpp"
#include <limits>
namespace core3d::debug {
inline std::array<bool,14> RunNativeWindingPlanProbe() {
    using namespace meshedit;
    std::array<bool,14> outcomes{};
    std::atomic_bool cancelled{false};
    try {
        // outward tetrahedron no-op
        {
            const std::vector<Point> vertices={{0,0,0}, {10,0,0}, {0,10,0}, {0,0,10}};
            const std::vector<std::array<unsigned,3>> indices={{0,2,1}, {0,1,3}, {1,2,3}, {2,0,3}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[0]=result==TopologyResult::Ready && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{}
                && plan.connectedComponents==1;
        }
        // one flipped non-seed tetrahedron face
        {
            const std::vector<Point> vertices={{0,0,0}, {10,0,0}, {0,10,0}, {0,0,10}};
            const std::vector<std::array<unsigned,3>> indices={{0,2,1}, {0,1,3}, {1,3,2}, {2,0,3}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[1]=result==TopologyResult::Ready && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{2}
                && plan.connectedComponents==1;
        }
        // seed controls consistency not outwardness
        {
            const std::vector<Point> vertices={{0,0,0}, {10,0,0}, {0,10,0}, {0,0,10}};
            const std::vector<std::array<unsigned,3>> indices={{0,1,2}, {0,1,3}, {1,2,3}, {2,0,3}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[2]=result==TopologyResult::Ready && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{1,2,3}
                && plan.connectedComponents==1;
        }
        // open square inconsistent diagonal traversal
        {
            const std::vector<Point> vertices={{0,0,0}, {10,0,0}, {10,10,0}, {0,10,0}};
            const std::vector<std::array<unsigned,3>> indices={{0,1,2}, {0,3,2}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[3]=result==TopologyResult::Ready && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{1}
                && plan.connectedComponents==1;
        }
        // opposite coincident face rejected
        {
            const std::vector<Point> vertices={{0,0,0}, {10,0,0}, {0,10,0}, {0,0,10}};
            const std::vector<std::array<unsigned,3>> indices={{0,2,1}, {0,1,3}, {1,2,3}, {2,0,3}, {0,1,2}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[4]=result==TopologyResult::NonManifold && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{}
                && plan.connectedComponents==0;
        }
        // same oriented duplicate rejected
        {
            const std::vector<Point> vertices={{0,0,0}, {10,0,0}, {0,10,0}, {0,0,10}};
            const std::vector<std::array<unsigned,3>> indices={{0,2,1}, {0,1,3}, {1,2,3}, {2,0,3}, {0,2,1}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[5]=result==TopologyResult::NonManifold && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{}
                && plan.connectedComponents==0;
        }
        // vertex-only bow tie rejected
        {
            const std::vector<Point> vertices={{0,0,0}, {10,0,0}, {0,10,0}, {-10,0,0}, {0,-10,0}};
            const std::vector<std::array<unsigned,3>> indices={{0,1,2}, {0,3,4}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[6]=result==TopologyResult::NonManifold && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{}
                && plan.connectedComponents==0;
        }
        // collinear distinct triangle rejected
        {
            const std::vector<Point> vertices={{0,0,0}, {1,0,0}, {2,0,0}};
            const std::vector<std::array<unsigned,3>> indices={{0,1,2}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[7]=result==TopologyResult::Invalid && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{}
                && plan.connectedComponents==0;
        }
        // three uses of one edge rejected
        {
            const std::vector<Point> vertices={{0,0,0}, {10,0,0}, {5,10,0}, {5,-10,0}, {5,0,10}};
            const std::vector<std::array<unsigned,3>> indices={{0,1,2}, {1,0,3}, {0,1,4}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[8]=result==TopologyResult::NonManifold && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{}
                && plan.connectedComponents==0;
        }
        // Mobius strip contradictory orientation cycle
        {
            const std::vector<Point> vertices={{1.75,0.0,-0.0}, {2.25,0.0,0.0}, {-0.9374999999999997,1.6237976320958225,-0.21650635094610965}, {-1.0624999999999996,1.8403039830419323,0.21650635094610965}, {-1.0625000000000009,-1.8403039830419314,-0.21650635094610968}, {-0.9375000000000009,-1.623797632095822,0.21650635094610968}};
            const std::vector<std::array<unsigned,3>> indices={{0,1,3}, {0,3,2}, {2,3,5}, {2,5,4}, {4,5,0}, {4,0,1}};
            std::vector<Triangle> input;
            for(const auto& tri:indices)input.push_back({vertices[tri[0]],vertices[tri[1]],vertices[tri[2]]});
            const auto original=input;
            WindingPlan plan;plan.reversedTriangles={123};plan.connectedComponents=123;
            const auto result=PlanConsistentWinding(input,plan,cancelled);
            outcomes[9]=result==TopologyResult::NonManifold && input==original
                && plan.reversedTriangles==std::vector<std::uint32_t>{}
                && plan.connectedComponents==0;
        }
        {
            WindingPlan plan;plan.reversedTriangles={1};plan.connectedComponents=1;
            outcomes[10]=PlanConsistentWinding({},plan,cancelled)==TopologyResult::Invalid
                && plan.reversedTriangles.empty() && plan.connectedComponents==0;
        }
        {
            const Triangle triangle={Point{0,0,0},Point{1,0,0},Point{0,1,0}};
            WindingPlan plan;
            outcomes[11]=PlanConsistentWinding(std::vector<Triangle>(4097,triangle),plan,cancelled)
                ==TopologyResult::TooLarge && plan.reversedTriangles.empty() && plan.connectedComponents==0;
        }
        {
            Triangle triangle={Point{0,0,0},Point{1,0,0},Point{0,1,0}};
            triangle[0][0]=std::numeric_limits<double>::quiet_NaN();
            WindingPlan plan;
            outcomes[12]=PlanConsistentWinding({triangle},plan,cancelled)==TopologyResult::Invalid
                && plan.reversedTriangles.empty() && plan.connectedComponents==0;
        }
        {
            const Triangle triangle={Point{0,0,0},Point{1,0,0},Point{0,1,0}};
            cancelled.store(true);
            WindingPlan plan;plan.reversedTriangles={1};plan.connectedComponents=1;
            outcomes[13]=PlanConsistentWinding({triangle},plan,cancelled)==TopologyResult::Cancelled
                && plan.reversedTriangles.empty() && plan.connectedComponents==0;
        }
    } catch(...) {}
    return outcomes;
}
} // namespace core3d::debug
#endif
