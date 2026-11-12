#pragma once

#include <BRepFilletAPI_MakeFillet.hxx>
#include <ChFiDS_ErrorStatus.hxx>
#include <set>

namespace core3d::fillet_failure {

// OCCT documents StartsolFailure as a possible oversized-radius condition,
// but it is not sufficient on its own. Admit only a complete, bounded and
// internally consistent set of faulty contours, with no vertex failure that
// could indicate a different topology problem.
inline bool IsClearanceRefusal(
    const BRepFilletAPI_MakeFillet& build) noexcept {
    try {
        const Standard_Integer contourCount = build.NbContours();
        const Standard_Integer faultyCount = build.NbFaultyContours();
        if (contourCount <= 0 || faultyCount <= 0
            || faultyCount > contourCount
            || build.NbFaultyVertices() != 0) {
            return false;
        }
        std::set<Standard_Integer> seen;
        for (Standard_Integer index = 1; index <= faultyCount; ++index) {
            const Standard_Integer contour = build.FaultyContour(index);
            if (contour < 1 || contour > contourCount
                || !seen.insert(contour).second
                || build.StripeStatus(contour) != ChFiDS_StartsolFailure) {
                return false;
            }
        }
        return seen.size() == static_cast<std::size_t>(faultyCount);
    } catch (...) {
        return false;
    }
}

} // namespace core3d::fillet_failure
