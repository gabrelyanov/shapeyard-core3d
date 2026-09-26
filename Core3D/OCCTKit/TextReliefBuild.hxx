#pragma once

// E5-3 retained emboss/deboss relief build. Consumes admitted E5-2 C1 curve
// records plus a planar host receipt, builds oriented loop faces, prisms them
// along the frame zAxis by the signed depth, and applies the retained detached
// boolean seam. Fully staged: nothing is published unless every gate passes.
// No OCAF label, command, executor, or UI authority.
#include "TextReliefDefinition.hxx"
#include "RetainedPartBoolean.hxx"
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepCheck_Wire.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <GProp_GProps.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom_BezierCurve.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_State.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Solid.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <cmath>
#include <set>
#include <vector>

namespace core3d::text_relief {

enum class BuildRefusal : std::uint8_t {
    None = 0, CurveLimit, ContourLimit, ComponentContourLimit, C1Refused,
    FrameMismatch, InconsistentLoops, OpenProfile, SelfIntersectingProfile,
    NestedContour, TinyTopology, NonPlanarHost, HostPlaneMismatch, StaleHost,
    DepthOutOfRange, InvalidUnitScale, BooleanFailure, IdentityCollision,
    RecipeRefused
};

struct ReliefBuildRequest {
    bounded_curve::UUID identitySeed{};
    retained_recipe::OwnerKey owner;
    bounded_curve::Frame frame;
    TopoDS_Shape hostSolid;
    TopoDS_Face hostFace;
    HostReceipt host;
    retained_recipe::DependencyRead hostCaptured, hostCurrent;
    std::vector<CurveRecord> curves;
    std::vector<ContourRecord> contours;
    Digest resourceSHA256{};
    ReliefOperation operation = ReliefOperation::Emboss;
    double depthMM = 0;
    double metersPerLocalUnit = 0;
};

struct ReliefBuildResult {
    BuildRefusal refusal = BuildRefusal::RecipeRefused;
    retained_part_boolean::Refusal booleanDetail = retained_part_boolean::Refusal::None;
    TopoDS_Shape solid;
    ReliefRecipe recipe;
    std::vector<std::uint8_t> recipeBytes;
};

namespace detail {
// Exact analytic ∮x dy in local units (CCW positive, y up). A line segment
// contributes cross(P0,P1)/2; a cubic Bézier contributes
// sum_i sum_k x_i (y_{k+1} - y_k) M[i][k] / 20 with
// M = [[10,4,1],[6,6,3],[3,6,6],[1,4,10]], since
// integral B_i^3 B_k^2 dt = C(3,i) C(2,k) / (6 C(5,i+k)) = M[i][k]/60 and the
// derivative carries a factor 3. Verified against BRepGProp to 1.4e-16.
inline double SignedLoopAreaLocal(
    const std::vector<std::vector<std::array<double, 3>>>& curvesPoles) noexcept {
    constexpr double M[4][3] = {{10, 4, 1}, {6, 6, 3}, {3, 6, 6}, {1, 4, 10}};
    double area = 0;
    for (const auto& poles : curvesPoles) {
        if (poles.size() == 2) {
            area += 0.5 * (poles[0][0] * poles[1][1] - poles[1][0] * poles[0][1]);
        } else if (poles.size() == 4) {
            for (int i = 0; i < 4; ++i)
                for (int k = 0; k < 3; ++k)
                    area += poles[i][0] * (poles[k + 1][1] - poles[k][1]) * M[i][k] / 20.0;
        }
    }
    return area;
}
} // namespace detail

inline ReliefBuildResult BuildPlanarRelief(const ReliefBuildRequest& request) noexcept {
    ReliefBuildResult result;
    const auto refuse = [&](BuildRefusal refusal) {
        result = {};
        result.refusal = refusal;
        return result;
    };
    const auto refuseBoolean = [&](retained_part_boolean::Refusal detail_) {
        result = {};
        result.refusal = BuildRefusal::BooleanFailure;
        result.booleanDetail = detail_;
        return result;
    };
    try {
        // 1. Admission caps first, then scalar and identity gates.
        if (request.contours.empty() || request.contours.size() > MaximumContours)
            return refuse(BuildRefusal::ContourLimit);
        {
            std::vector<std::size_t> perComponent;
            for (const ContourRecord& contour : request.contours) {
                if (contour.component >= perComponent.size())
                    perComponent.resize(contour.component + 1);
                if (++perComponent[contour.component] > MaximumContoursPerComponent)
                    return refuse(BuildRefusal::ComponentContourLimit);
            }
        }
        if (request.curves.empty() || request.curves.size() > MaximumCurves)
            return refuse(BuildRefusal::CurveLimit);
        const double depth = std::abs(request.depthMM);
        if ((request.operation != ReliefOperation::Emboss
                && request.operation != ReliefOperation::Deboss)
            || !bounded_curve::Finite(request.depthMM) || depth < MinimumReliefDepthMM
            || depth > MaximumReliefDepthMM
            || (request.operation == ReliefOperation::Emboss && request.depthMM <= 0)
            || (request.operation == ReliefOperation::Deboss && request.depthMM >= 0))
            return refuse(BuildRefusal::DepthOutOfRange);
        if (!bounded_curve::Finite(request.metersPerLocalUnit)
            || request.metersPerLocalUnit <= 0 || request.metersPerLocalUnit > 1)
            return refuse(BuildRefusal::InvalidUnitScale);
        if (!bounded_curve::Nonzero(request.identitySeed)
            || !retained_recipe::Valid(request.owner) || !Valid(request.host))
            return refuse(BuildRefusal::RecipeRefused);
        const double scale = request.metersPerLocalUnit;

        // 2. Decode every C1 record; the contour partition, not the record's
        // own component/contour/segment fields, is authoritative here.
        std::vector<bounded_curve::Value> decoded(request.curves.size());
        std::vector<Digest> curveDigests(request.curves.size());
        std::set<bounded_curve::UUID> features;
        const auto sameFrame = [](const bounded_curve::Frame& a,
                                  const bounded_curve::Frame& b) {
            return a.identifier == b.identifier && a.revision == b.revision
                && a.origin == b.origin && a.xAxis == b.xAxis && a.yAxis == b.yAxis
                && a.zAxis == b.zAxis && a.handedness == b.handedness;
        };
        for (std::size_t index = 0; index < request.curves.size(); ++index) {
            bounded_curve::Value value;
            if (!bounded_curve::Decode(request.curves[index].canonicalBytes, value))
                return refuse(BuildRefusal::C1Refused);
            if (!bounded_curve::Nonzero(value.feature) || !features.insert(value.feature).second)
                return refuse(BuildRefusal::IdentityCollision);
            const auto& definition = value.definition;
            const auto multiplicity = std::uint8_t(definition.degree + 1);
            if (definition.domain != bounded_curve::Domain::Sketch2D
                || (definition.degree != 1 && definition.degree != 3)
                || definition.knots.size() != 2
                || definition.knots[0].value != 0 || definition.knots[1].value != 1
                || definition.knots[0].multiplicity != multiplicity
                || definition.knots[1].multiplicity != multiplicity
                || (!definition.weights.empty()
                    && definition.weights.size() != definition.controlPoints.size()))
                return refuse(BuildRefusal::C1Refused);
            if (!sameFrame(definition.frame, request.frame))
                return refuse(BuildRefusal::FrameMismatch);
            if (!bounded_curve::Hash(request.curves[index].canonicalBytes,
                                     bounded_curve::MaximumDefinitionBytes, curveDigests[index]))
                return refuse(BuildRefusal::C1Refused);
            decoded[index] = std::move(value);
        }

        // 3. The loop partition must tile [0, curves.size()) exactly in order.
        {
            std::size_t cursor = 0;
            for (const ContourRecord& contour : request.contours) {
                if (contour.curveCount == 0 || contour.firstCurve != cursor
                    || cursor + contour.curveCount > decoded.size())
                    return refuse(BuildRefusal::InconsistentLoops);
                cursor += contour.curveCount;
            }
            if (cursor != decoded.size()) return refuse(BuildRefusal::InconsistentLoops);
        }

        // 4. Chained endpoint closure in local coordinates, and no degenerate
        // zero-length curve anywhere.
        const auto poles = [&](std::size_t curve) -> const std::vector<bounded_curve::ControlPoint>& {
            return decoded[curve].definition.controlPoints;
        };
        const auto sameLocalPoint = [](const std::array<double, 3>& a,
                                       const std::array<double, 3>& b) {
            return std::abs(a[0] - b[0]) <= 1e-7 && std::abs(a[1] - b[1]) <= 1e-7
                && std::abs(a[2] - b[2]) <= 1e-7;
        };
        for (const ContourRecord& contour : request.contours) {
            for (std::size_t k = 0; k < contour.curveCount; ++k) {
                const auto& current = poles(contour.firstCurve + k);
                const auto& next = poles(contour.firstCurve + (k + 1) % contour.curveCount);
                if (sameLocalPoint(current.front().local, current.back().local)
                    || !sameLocalPoint(current.back().local, next.front().local))
                    return refuse(BuildRefusal::OpenProfile);
            }
        }

        // 5. Host receipt: planar face on the frame plane, a still-valid host
        // solid, unchanged shape bytes, and a current semantic read set.
        BRepAdaptor_Surface hostSurface(request.hostFace);
        if (hostSurface.GetType() != GeomAbs_Plane)
            return refuse(BuildRefusal::NonPlanarHost);
        const gp_Pln hostPlane = hostSurface.Plane();
        const gp_Dir hostNormal = hostPlane.Axis().Direction();
        const gp_Vec frameZ(request.frame.zAxis[0], request.frame.zAxis[1],
                            request.frame.zAxis[2]);
        const gp_Pnt frameOrigin(request.frame.origin[0], request.frame.origin[1],
                                 request.frame.origin[2]);
        const double alignment = hostNormal.Dot(gp_Dir(frameZ));
        if (std::abs(alignment) < 1 - 1e-9 || hostPlane.Distance(frameOrigin) > 1e-7)
            return refuse(BuildRefusal::HostPlaneMismatch);
        if (!retained_part_boolean::IsOneValidForwardSolid(request.hostSolid))
            return refuseBoolean(retained_part_boolean::Refusal::InvalidInput);
        {
            std::string exactBytes;
            Digest actual{};
            if (!retained_part_boolean::ExactShapeBytes(request.hostSolid, exactBytes)
                || exactBytes.empty())
                return refuseBoolean(retained_part_boolean::Refusal::InvalidInput);
            const std::vector<std::uint8_t> bytes(exactBytes.begin(), exactBytes.end());
            if (!bounded_curve::Hash(bytes, 64 * MaximumRecipeBytes, actual)
                || actual != request.host.shapeDigest)
                return refuse(BuildRefusal::StaleHost);
        }
        if (!retained_part_boolean::ValidOperandRead(request.hostCaptured)
            || !retained_part_boolean::SameOperandRead(request.hostCaptured,
                                                       request.hostCurrent))
            return refuse(BuildRefusal::StaleHost);

        // 6. Edges, closed wires, exact winding area, containment roles, and
        // self-intersection discipline mirroring ProfileCurveFace.
        const auto modelPoint = [&](const std::array<double, 3>& local) {
            return gp_Pnt(
                request.frame.origin[0] + scale * (local[0] * request.frame.xAxis[0]
                    + local[1] * request.frame.yAxis[0] + local[2] * request.frame.zAxis[0]),
                request.frame.origin[1] + scale * (local[0] * request.frame.xAxis[1]
                    + local[1] * request.frame.yAxis[1] + local[2] * request.frame.zAxis[1]),
                request.frame.origin[2] + scale * (local[0] * request.frame.xAxis[2]
                    + local[1] * request.frame.yAxis[2] + local[2] * request.frame.zAxis[2]));
        };
        std::vector<TopoDS_Edge> edges(decoded.size());
        for (std::size_t index = 0; index < decoded.size(); ++index) {
            const auto& definition = decoded[index].definition;
            if (definition.degree == 1) {
                BRepBuilderAPI_MakeEdge made(modelPoint(definition.controlPoints[0].local),
                                             modelPoint(definition.controlPoints[1].local));
                if (!made.IsDone()) return refuse(BuildRefusal::OpenProfile);
                edges[index] = made.Edge();
            } else {
                TColgp_Array1OfPnt polesArray(1, 4);
                for (int pole = 0; pole < 4; ++pole)
                    polesArray.SetValue(pole + 1, modelPoint(definition.controlPoints[pole].local));
                Handle(Geom_BezierCurve) curve;
                if (definition.weights.empty()) {
                    curve = new Geom_BezierCurve(polesArray);
                } else {
                    TColStd_Array1OfReal weightsArray(1, 4);
                    for (int pole = 0; pole < 4; ++pole)
                        weightsArray.SetValue(pole + 1, definition.weights[pole]);
                    curve = new Geom_BezierCurve(polesArray, weightsArray);
                }
                BRepBuilderAPI_MakeEdge made(curve, 0.0, 1.0);
                if (!made.IsDone()) return refuse(BuildRefusal::OpenProfile);
                edges[index] = made.Edge();
            }
        }
        struct Loop {
            TopoDS_Wire authoredWire, positiveWire;
            TopoDS_Face probeFace;
            std::vector<TopoDS_Edge> edges;
            gp_Pnt representative;
            double signedArea = 0;
            LoopRole role = LoopRole::Outer;
            std::size_t parent = 0; // recorded index of the containing outer
        };
        std::vector<Loop> loops(request.contours.size());
        constexpr double localClearance = 1e-3; // mm, before unit scaling
        const double clearance = localClearance * scale; // model units
        const auto distant = [&](const TopoDS_Shape& a, const TopoDS_Shape& b) {
            BRepExtrema_DistShapeShape distance(a, b);
            return distance.IsDone() && distance.NbSolution() > 0
                && std::isfinite(distance.Value()) && distance.Value() >= clearance;
        };
        const gp_Pln framePlane(frameOrigin, gp_Dir(frameZ));
        for (std::size_t loopIndex = 0; loopIndex < loops.size(); ++loopIndex) {
            const ContourRecord& contour = request.contours[loopIndex];
            Loop& loop = loops[loopIndex];
            BRepBuilderAPI_MakeWire maker;
            std::vector<std::vector<std::array<double, 3>>> loopPoles;
            for (std::size_t k = 0; k < contour.curveCount; ++k) {
                maker.Add(edges[contour.firstCurve + k]);
                if (!maker.IsDone()) return refuse(BuildRefusal::OpenProfile);
                loop.edges.push_back(edges[contour.firstCurve + k]);
                std::vector<std::array<double, 3>> curvePoles;
                for (const auto& point : poles(contour.firstCurve + k))
                    curvePoles.push_back(point.local);
                loopPoles.push_back(curvePoles);
            }
            loop.authoredWire = maker.Wire();
            if (loop.authoredWire.IsNull()
                || BRepCheck_Wire(loop.authoredWire).Closed() != BRepCheck_NoError)
                return refuse(BuildRefusal::OpenProfile);
            loop.signedArea = detail::SignedLoopAreaLocal(loopPoles);
            if (!std::isfinite(loop.signedArea) || std::abs(loop.signedArea) < 1e-6)
                return refuse(BuildRefusal::TinyTopology);
            // Adjacent edges meet by design; nonadjacent boundaries keep clearance.
            for (std::size_t i = 0; i < loop.edges.size(); ++i)
                for (std::size_t j = i + 1; j < loop.edges.size(); ++j) {
                    if (j == i + 1 || (i == 0 && j + 1 == loop.edges.size())) continue;
                    if (!distant(loop.edges[i], loop.edges[j]))
                        return refuse(BuildRefusal::SelfIntersectingProfile);
                }
            loop.positiveWire = loop.signedArea > 0
                ? loop.authoredWire : TopoDS::Wire(loop.authoredWire.Reversed());
            loop.representative = modelPoint(poles(contour.firstCurve).front().local);
        }
        // Distinct loops never touch; containment then decides roles.
        for (std::size_t i = 0; i < loops.size(); ++i)
            for (std::size_t j = i + 1; j < loops.size(); ++j)
                if (!distant(loops[i].positiveWire, loops[j].positiveWire))
                    return refuse(BuildRefusal::TinyTopology);
        for (Loop& loop : loops) {
            BRepBuilderAPI_MakeFace face(framePlane, loop.positiveWire, Standard_True);
            if (!face.IsDone()) return refuse(BuildRefusal::SelfIntersectingProfile);
            loop.probeFace = face.Face();
            BRepCheck_Wire wireCheck(loop.positiveWire);
            TopoDS_Edge firstIntersection, secondIntersection;
            if (wireCheck.SelfIntersect(loop.probeFace, firstIntersection,
                                        secondIntersection) != BRepCheck_NoError
                || !BRepCheck_Analyzer(loop.probeFace, Standard_True).IsValid())
                return refuse(BuildRefusal::SelfIntersectingProfile);
        }
        for (std::size_t i = 0; i < loops.size(); ++i) {
            std::size_t depthCount = 0, container = 0;
            for (std::size_t j = 0; j < loops.size(); ++j) {
                if (i == j) continue;
                BRepClass_FaceClassifier classifier(loops[j].probeFace,
                                                    loops[i].representative, 1e-7);
                if (classifier.State() == TopAbs_IN) { ++depthCount; container = j; }
            }
            if (depthCount >= 2) return refuse(BuildRefusal::NestedContour);
            loops[i].role = depthCount == 0 ? LoopRole::Outer : LoopRole::Hole;
            loops[i].parent = container;
        }
        {
            bool sawOuter = false;
            for (const Loop& loopValue : loops) {
                if (loopValue.role == LoopRole::Hole && !sawOuter)
                    return refuse(BuildRefusal::NestedContour);
                sawOuter = sawOuter || loopValue.role == LoopRole::Outer;
            }
            if (!sawOuter) return refuse(BuildRefusal::NestedContour);
        }

        // Assemble the retained recipe now that roles are known; the boolean
        // read-set digests reference these exact canonical bytes.
        ReliefRecipe recipe;
        recipe.owner = request.owner;
        recipe.host = request.host;
        recipe.operation = request.operation;
        recipe.depthMM = request.depthMM;
        recipe.metersPerLocalUnit = request.metersPerLocalUnit;
        recipe.resourceSHA256 = request.resourceSHA256;
        for (std::size_t index = 0; index < decoded.size(); ++index)
            recipe.curves.push_back({decoded[index].feature, curveDigests[index]});
        for (std::size_t loopIndex = 0; loopIndex < loops.size(); ++loopIndex) {
            const ContourRecord& contour = request.contours[loopIndex];
            recipe.loops.push_back({std::uint32_t(contour.component),
                std::uint32_t(contour.contour), std::uint32_t(contour.firstCurve),
                std::uint32_t(contour.curveCount), loops[loopIndex].role});
        }
        recipe.feature = DeriveReliefFeature(request.identitySeed, request.host,
            request.resourceSHA256, request.operation, request.depthMM,
            request.metersPerLocalUnit, curveDigests);
        std::vector<std::uint8_t> recipeBytes;
        if (!bounded_curve::Nonzero(recipe.feature) || !Encode(recipe, recipeBytes))
            return refuse(BuildRefusal::RecipeRefused);

        // 7. One face-with-holes per outer, then a signed prism. The prism is
        // extended past the host plane by a small deterministic overlap and
        // shifted so the relief boundary sits exactly on the frame plane: a
        // relief starting flush on the host face is then never a tangent-only
        // boolean, and the overlap lies inside the host, never changing the
        // resulting volume.
        const double depthModel = request.depthMM * scale; // signed, model units
        const double overlap = std::max(std::abs(depthModel) * 1e-3, 1e-6);
        const double extension = request.operation == ReliefOperation::Emboss
            ? overlap : -overlap;
        const gp_Vec sweep(frameZ.X() * (depthModel + extension),
                           frameZ.Y() * (depthModel + extension),
                           frameZ.Z() * (depthModel + extension));
        gp_Trsf shift;
        shift.SetTranslation(gp_Vec(frameZ.X() * -extension,
                                    frameZ.Y() * -extension,
                                    frameZ.Z() * -extension));
        std::vector<TopoDS_Solid> prisms;
        for (std::size_t loopIndex = 0; loopIndex < loops.size(); ++loopIndex) {
            if (loops[loopIndex].role != LoopRole::Outer) continue;
            BRepBuilderAPI_MakeFace face(framePlane, loops[loopIndex].positiveWire,
                                         Standard_True);
            if (!face.IsDone())
                return refuseBoolean(retained_part_boolean::Refusal::BuildFailed);
            for (std::size_t holeIndex = 0; holeIndex < loops.size(); ++holeIndex)
                if (loops[holeIndex].role == LoopRole::Hole
                    && loops[holeIndex].parent == loopIndex) {
                    face.Add(TopoDS::Wire(loops[holeIndex].positiveWire.Reversed()));
                    if (!face.IsDone())
                        return refuseBoolean(retained_part_boolean::Refusal::BuildFailed);
                }
            if (!BRepCheck_Analyzer(face.Face(), Standard_True).IsValid())
                return refuseBoolean(retained_part_boolean::Refusal::BuildFailed);
            BRepPrimAPI_MakePrism prism(face.Face(), sweep);
            if (!prism.IsDone())
                return refuseBoolean(retained_part_boolean::Refusal::BuildFailed);
            BRepBuilderAPI_Transform moved(prism.Shape(), shift, Standard_True);
            if (!moved.IsDone())
                return refuseBoolean(retained_part_boolean::Refusal::BuildFailed);
            std::size_t solids = 0;
            TopoDS_Solid prismSolid;
            for (TopExp_Explorer it(moved.Shape(), TopAbs_SOLID); it.More(); it.Next()) {
                if (++solids > 1) break;
                prismSolid = TopoDS::Solid(it.Current());
            }
            if (solids != 1 || prismSolid.IsNull()
                || !BRepCheck_Analyzer(prismSolid).IsValid()
                || retained_part_boolean::Volume(prismSolid) <= 0)
                return refuseBoolean(retained_part_boolean::Refusal::BuildFailed);
            prisms.push_back(TopoDS::Solid(prismSolid.Oriented(TopAbs_FORWARD)));
        }

        // 8. Sequential retained detached booleans in recorded outer order.
        const auto taggedDigest = [&](const char* tag, Digest& output) {
            std::vector<std::uint8_t> material;
            while (*tag) material.push_back(std::uint8_t(*tag++));
            material.insert(material.end(), recipeBytes.begin(), recipeBytes.end());
            return bounded_curve::Hash(material, MaximumRecipeBytes + 64, output);
        };
        retained_recipe::DependencyRead rightSource;
        rightSource.locator = {request.owner, recipe.feature, recipe.feature};
        if (!taggedDigest("SYTR-geometry", rightSource.geometry)
            || !taggedDigest("SYTR-recipe", rightSource.recipe)
            || !taggedDigest("SYTR-placement", rightSource.placement)
            || !taggedDigest("SYTR-material", rightSource.material)
            || !taggedDigest("SYTR-groups", rightSource.groups))
            return refuse(BuildRefusal::RecipeRefused);
        retained_part_boolean::OperandReadSet reads;
        reads.leftSource = request.hostCaptured;
        reads.rightSource = rightSource;
        const auto operation = request.operation == ReliefOperation::Emboss
            ? retained_part_boolean::Operation::Union
            : retained_part_boolean::Operation::Subtract;
        TopoDS_Shape current = request.hostSolid;
        TopoDS_Shape final;
        for (const TopoDS_Solid& prism : prisms) {
            const auto candidate = retained_part_boolean::BuildDetachedCandidate(
                operation, current, prism, 1e-7, reads, reads);
            if (!candidate.admitted()) return refuseBoolean(candidate.refusal);
            std::size_t solids = 0;
            TopoDS_Solid extracted;
            for (TopExp_Explorer it(candidate.solid, TopAbs_SOLID); it.More(); it.Next()) {
                if (++solids > 1) break;
                extracted = TopoDS::Solid(it.Current());
            }
            if (solids != 1 || extracted.IsNull()
                || !BRepCheck_Analyzer(extracted).IsValid()
                || retained_part_boolean::Volume(extracted) <= 0)
                return refuseBoolean(retained_part_boolean::Refusal::BuildFailed);
            current = TopoDS::Solid(extracted.Oriented(TopAbs_FORWARD));
            final = candidate.solid;
        }

        // 9. Publish only on complete success.
        result.refusal = BuildRefusal::None;
        result.booleanDetail = retained_part_boolean::Refusal::None;
        result.solid = final;
        result.recipe = std::move(recipe);
        result.recipeBytes = std::move(recipeBytes);
        return result;
    } catch (...) {
        result = {};
        result.refusal = BuildRefusal::BooleanFailure;
        result.booleanDetail = retained_part_boolean::Refusal::BuildFailed;
        return result;
    }
}
} // namespace core3d::text_relief
