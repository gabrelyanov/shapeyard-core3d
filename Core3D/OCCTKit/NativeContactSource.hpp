#pragma once
// Private value contract. No live OCCT references may enter detached work.
#include "NativeTriangleContacts.hpp"
#include <cstdint>
#include <bit>
#include <string>

namespace core3d::meshcheck {
struct ContactSourceIdentity {
    std::string entityIdentifier;
    std::string definitionIdentifier;
    std::string publicationSourceIdentifier;
    std::uint64_t documentGeneration=0;
    std::uint64_t modelRevision=0;
    std::uint64_t geometryRevision=0;
};
enum class ContactSourceStatus {
    Ready, Unsupported, InvalidGeometry, ResourceLimit, Cancelled, TimedOut,
    StaleSource, InternalFailure
};
struct ContactSourceCapture {
    ContactSourceIdentity identity;
    // Additional native transaction guard; renderer Float fingerprints alone
    // cannot detect a sub-Float change to authoritative Double coordinates.
    std::int64_t documentTime=0;
    std::size_t storedNodeCount=0;
    std::vector<Triangle> triangles;
    std::array<double,12> facePlacement{};
    std::array<double,16> worldFromObject{};
    int faceOrientation=0;
    std::vector<Point> storedNodes;
    // Original OCCT one-based indices. Public pairs remain zero-based triangle
    // ordinals; these index triples are private identity validation only.
    std::vector<std::array<std::uint32_t,3>> triangleNodeIDs;
};
inline bool SameContactSource(const ContactSourceCapture& a,
                              const ContactSourceCapture& b) noexcept {
    const auto& x=a.identity;const auto& y=b.identity;
    if (x.entityIdentifier!=y.entityIdentifier
        || x.definitionIdentifier!=y.definitionIdentifier
        || x.publicationSourceIdentifier!=y.publicationSourceIdentifier
        || x.documentGeneration!=y.documentGeneration
        || x.modelRevision!=y.modelRevision || x.geometryRevision!=y.geometryRevision
        || a.documentTime!=b.documentTime || a.storedNodeCount!=b.storedNodeCount
        || a.storedNodes.size()!=b.storedNodes.size()
        || a.triangles.size()!=b.triangles.size()
        || a.faceOrientation!=b.faceOrientation
        || a.triangleNodeIDs!=b.triangleNodeIDs) return false;
    for (std::size_t i=0;i<a.facePlacement.size();++i)
        if (std::bit_cast<std::uint64_t>(a.facePlacement[i])
            !=std::bit_cast<std::uint64_t>(b.facePlacement[i])) return false;
    for (std::size_t i=0;i<a.worldFromObject.size();++i)
        if (std::bit_cast<std::uint64_t>(a.worldFromObject[i])
            !=std::bit_cast<std::uint64_t>(b.worldFromObject[i])) return false;
    for (std::size_t i=0;i<a.storedNodes.size();++i)
        for (int k=0;k<3;++k)
            if (std::bit_cast<std::uint64_t>(a.storedNodes[i][k])
                !=std::bit_cast<std::uint64_t>(b.storedNodes[i][k])) return false;
    // Comparing every exact triangle coordinate also protects this contract if
    // a later capture changes how stored indices are interpreted.
    for (std::size_t i=0;i<a.triangles.size();++i)
        for (int j=0;j<3;++j)for (int k=0;k<3;++k)
            if (std::bit_cast<std::uint64_t>(a.triangles[i][j][k])
                !=std::bit_cast<std::uint64_t>(b.triangles[i][j][k])) return false;
    return true;
}
} // namespace core3d::meshcheck
