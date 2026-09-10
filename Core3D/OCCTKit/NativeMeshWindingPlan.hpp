#pragma once
// Private, attribute-agnostic triangle winding proposal. NOT native edit
// authority or a document operation. Publication must retain exact corner
// UV/material/frame ownership and use an ordinary native recovery ledger.
#include "EditableMeshTopology.hpp"

namespace core3d::meshedit {
struct WindingPlan {
    // Zero-based immutable input triangle ordinals. Swapping corners1/2 makes
    // edge traversal consistent. No vertex welding or coordinate change.
    std::vector<std::uint32_t> reversedTriangles;
    std::size_t connectedComponents=0;
};
inline TopologyResult PlanConsistentWinding(const std::vector<Triangle>& input,
    WindingPlan& output, const std::atomic_bool& cancelled) noexcept {
    output={};
    if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
    if(input.empty())return TopologyResult::Invalid;
    if(input.size()>4096)return TopologyResult::TooLarge;
    try {
        struct Use {std::uint32_t triangle;bool forward;};
        std::map<Point,std::uint32_t> vertices;
        std::map<std::array<std::uint32_t,2>,std::vector<Use>> edges;
        std::set<std::array<std::uint32_t,3>> uniqueTriangles;
        for(std::size_t t=0;t<input.size();++t) {
            if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
            std::array<std::uint32_t,3> ids{};
            for(int k=0;k<3;++k) {
                for(double c:input[t][k])
                    if(!std::isfinite(c)||std::abs(c)>1.e6)return TopologyResult::Invalid;
                auto found=vertices.find(input[t][k]);
                if(found==vertices.end()) {
                    const auto id=static_cast<std::uint32_t>(vertices.size());
                    found=vertices.emplace(input[t][k],id).first;
                }
                ids[k]=found->second;
            }
            if(ids[0]==ids[1]||ids[1]==ids[2]||ids[0]==ids[2])return TopologyResult::Invalid;
            auto sorted=ids;std::sort(sorted.begin(),sorted.end());
            // Coincident double surfaces cannot be fixed by choosing a winding.
            if(!uniqueTriangles.insert(sorted).second)return TopologyResult::NonManifold;
            for(int k=0;k<3;++k) {
                const auto a=ids[k],b=ids[(k+1)%3];
                auto& uses=edges[{std::min(a,b),std::max(a,b)}];
                if(uses.size()==2)return TopologyResult::NonManifold;
                uses.push_back({static_cast<std::uint32_t>(t),a<b});
            }
        }
        struct Neighbor {std::uint32_t triangle;bool reverseRelative;};
        std::vector<std::vector<Neighbor>> neighbors(input.size());
        for(const auto& entry:edges) {
            const auto& uses=entry.second;
            if(uses.size()!=2)continue;
            const bool reverseRelative=uses[0].forward==uses[1].forward;
            neighbors[uses[0].triangle].push_back({uses[1].triangle,reverseRelative});
            neighbors[uses[1].triangle].push_back({uses[0].triangle,reverseRelative});
        }
        // Preserve the earliest input triangle orientation of each connected
        // component. This is consistency only, not an outward/inside decision.
        std::vector<int> reversed(input.size(),-1);
        WindingPlan result;
        for(std::size_t seed=0;seed<input.size();++seed) {
            if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
            if(reversed[seed]!=-1)continue;
            ++result.connectedComponents;
            reversed[seed]=0;
            std::vector<std::uint32_t> queue={static_cast<std::uint32_t>(seed)};
            for(std::size_t q=0;q<queue.size();++q) {
                if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
                const auto t=queue[q];
                for(const auto& next:neighbors[t]) {
                    const int wanted=reversed[t]^int(next.reverseRelative);
                    if(reversed[next.triangle]==-1) {
                        reversed[next.triangle]=wanted;queue.push_back(next.triangle);
                    } else if(reversed[next.triangle]!=wanted) {
                        return TopologyResult::NonManifold;
                    }
                }
            }
        }
        auto candidate=input;
        for(std::size_t t=0;t<input.size();++t)if(reversed[t]) {
            result.reversedTriangles.push_back(static_cast<std::uint32_t>(t));
            std::swap(candidate[t][1],candidate[t][2]);
        }
        // Reuse the existing strict postcondition without weakening the current
        // picker: zero-area geometry, bow-tie fans and bad winding still reject.
        Topology checked;
        const auto status=Analyze(candidate,checked,cancelled);
        if(status!=TopologyResult::Ready)return status;
        if(cancelled.load(std::memory_order_relaxed))return TopologyResult::Cancelled;
        output=std::move(result);return TopologyResult::Ready;
    } catch(...) {output={};return TopologyResult::Invalid;}
}
} // namespace core3d::meshedit
