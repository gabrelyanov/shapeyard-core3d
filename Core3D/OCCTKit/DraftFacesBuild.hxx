#pragma once

#include "DraftFacesDefinition.hxx"
#include <BOPAlgo_ArgumentAnalyzer.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_DraftAngle.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS.hxx>
#include <atomic>
#include <cmath>
#include <locale>
#include <set>
#include <sstream>

namespace core3d::draft_faces {

struct AddObservation final {
    UUID face{};
    bool addDone = false;
    int status = -1;
};

struct AnalyticSection final {
    UUID face{};
    double pullOffsetLocal = 0;
    double expectedNormalOffsetLocal = 0;
    double measuredNormalOffsetLocal = 0;
};

struct BuildEvidence final {
    std::vector<AddObservation> adds;
    std::vector<AnalyticSection> sections;
    double measuredSignedAngleRadians = 0;
    std::size_t neutralSectionEdges = 0;
    double neutralSectionLengthLocal = 0;
    std::size_t connectedSolids = 0;
    bool sourceBytesUnchanged = false;
    bool candidateBytesDistinct = false;
};

struct BuildResult final {
    Refusal refusal = Refusal::BuildFailure;
    TopoDS_Shape solid;
    BuildEvidence evidence;
    bool built() const noexcept { return refusal == Refusal::None && !solid.IsNull(); }
};

namespace detail {
inline gp_Pnt point(const std::array<double, 3>& value) {
    return {value[0], value[1], value[2]};
}
inline gp_Dir direction(const std::array<double, 3>& value) {
    return {value[0], value[1], value[2]};
}
inline std::array<double, 3> components(const gp_Dir& value) noexcept {
    return {{value.X(), value.Y(), value.Z()}};
}
inline gp_Dir outwardPlaneNormal(const TopoDS_Face& face,
                                 const BRepAdaptor_Surface& surface) {
    gp_Dir normal = surface.Plane().Axis().Direction();
    if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
    return normal;
}
inline SurfaceKind kind(GeomAbs_SurfaceType value) noexcept {
    if (value == GeomAbs_Plane) return SurfaceKind::Planar;
    if (value == GeomAbs_Cylinder) return SurfaceKind::Cylindrical;
    if (value == GeomAbs_Cone) return SurfaceKind::Conical;
    return SurfaceKind(0);
}
inline std::size_t boundaryEdges(const TopoDS_Face& face) {
    std::size_t count = 0;
    for (TopExp_Explorer explorer(face, TopAbs_EDGE); explorer.More(); explorer.Next()) ++count;
    return count;
}
inline bool pointOnFace(const TopoDS_Face& face, const gp_Pnt& sample,
                        double tolerance) {
    BRepExtrema_DistShapeShape distance(BRepBuilderAPI_MakeVertex(sample).Vertex(), face);
    distance.Perform();
    return distance.IsDone() && distance.NbSolution() > 0
        && distance.Value() <= tolerance;
}

inline bool matches(const TopoDS_Face& face, const FaceIntent& intent,
                    double tolerance) {
    BRepAdaptor_Surface surface(face);
    if (kind(surface.GetType()) != intent.surface
        || boundaryEdges(face) != intent.expectedBoundaryEdges
        || !pointOnFace(face, point(intent.sampleOnNeutralLocal), tolerance)) return false;
    if (intent.surface == SurfaceKind::Planar) {
        const gp_Dir normal = outwardPlaneNormal(face, surface);
        if (normal.Dot(direction(intent.outwardNormalLocal)) < 1.0 - 1e-10) return false;
    }
    return true;
}

inline bool resolve(const TopoDS_Shape& source, const FaceIntent& intent,
                    double tolerance, TopoDS_Face& output, Refusal& refusal) {
    output.Nullify(); std::size_t count = 0;
    for (TopExp_Explorer explorer(source, TopAbs_FACE); explorer.More(); explorer.Next()) {
        const TopoDS_Face candidate = TopoDS::Face(explorer.Current());
        if (matches(candidate, intent, tolerance)) { output = candidate; ++count; }
    }
    if (count == 0) { refusal = Refusal::AnchorMissing; return false; }
    if (count != intent.expectedCardinality) {
        output.Nullify(); refusal = Refusal::AnchorAmbiguous; return false;
    }
    return true;
}

inline bool exactShapeBytes(const TopoDS_Shape& shape,
                            std::vector<std::uint8_t>& bytes) {
    bytes.clear(); std::ostringstream stream; stream.imbue(std::locale::classic());
    BRepTools::Write(shape, stream, Standard_False, Standard_False,
                     TopTools_FormatVersion_VERSION_3);
    if (!stream.good()) return false;
    const std::string value = stream.str();
    bytes.assign(value.begin(), value.end()); return !bytes.empty();
}

struct PointLess final {
    bool operator()(const gp_Pnt& a, const gp_Pnt& b) const noexcept {
        if (a.X() != b.X()) return a.X() < b.X();
        if (a.Y() != b.Y()) return a.Y() < b.Y();
        return a.Z() < b.Z();
    }
};

struct SectionWitness final {
    std::size_t edges = 0;
    double length = 0;
    std::vector<gp_Pnt> vertices;
};

inline bool sectionWitness(const TopoDS_Shape& shape, const gp_Pln& plane,
                           double tolerance, SectionWitness& output) {
    output = {};
    BRepAlgoAPI_Section section(shape, plane, Standard_False);
    section.Approximation(Standard_False); section.Build();
    if (!section.IsDone() || section.Shape().IsNull()) return false;
    for (TopExp_Explorer edge(section.Shape(), TopAbs_EDGE); edge.More(); edge.Next()) {
        GProp_GProps property; BRepGProp::LinearProperties(edge.Current(), property);
        if (!std::isfinite(property.Mass()) || property.Mass() <= tolerance) return false;
        output.length += property.Mass(); ++output.edges;
    }
    for (TopExp_Explorer vertex(section.Shape(), TopAbs_VERTEX); vertex.More(); vertex.Next()) {
        const gp_Pnt candidate = BRep_Tool::Pnt(TopoDS::Vertex(vertex.Current()));
        const bool duplicate = std::any_of(output.vertices.begin(), output.vertices.end(),
            [&](const gp_Pnt& value) { return value.Distance(candidate) <= tolerance; });
        if (!duplicate) output.vertices.push_back(candidate);
    }
    std::sort(output.vertices.begin(), output.vertices.end(), PointLess{});
    return output.edges > 0 && output.vertices.size() >= 3 && std::isfinite(output.length);
}

inline bool sameSection(const SectionWitness& a, const SectionWitness& b,
                        double tolerance) noexcept {
    if (a.edges != b.edges || a.vertices.size() != b.vertices.size()
        || std::abs(a.length - b.length) > tolerance * std::max<std::size_t>(1, a.edges)) {
        return false;
    }
    for (std::size_t index = 0; index < a.vertices.size(); ++index)
        if (a.vertices[index].Distance(b.vertices[index]) > tolerance) return false;
    return true;
}

inline bool selfIntersectionFree(const TopoDS_Shape& shape) {
    BOPAlgo_ArgumentAnalyzer analyzer;
    analyzer.SetShape1(shape); analyzer.SetRunParallel(Standard_False);
    analyzer.StopOnFirstFaulty() = Standard_True;
    analyzer.ArgumentTypeMode() = Standard_False; analyzer.SelfInterMode() = Standard_True;
    analyzer.SmallEdgeMode() = Standard_False; analyzer.RebuildFaceMode() = Standard_False;
    analyzer.TangentMode() = Standard_False; analyzer.MergeVertexMode() = Standard_False;
    analyzer.MergeEdgeMode() = Standard_False; analyzer.ContinuityMode() = Standard_False;
    analyzer.CurveOnSurfaceMode() = Standard_False; analyzer.Perform();
    return !analyzer.HasErrors() && !analyzer.HasWarnings() && !analyzer.HasFaulty();
}

inline bool candidateFace(const TopoDS_Shape& candidate, const FaceIntent& intent,
                          const Definition& definition, double tolerance,
                          TopoDS_Face& output, gp_Dir& outputNormal) {
    output.Nullify(); std::size_t count = 0;
    const gp_Dir original = direction(intent.outwardNormalLocal);
    const gp_Dir pull = direction(definition.pullDirectionLocal);
    gp_Vec expected(original); expected.Multiply(std::cos(definition.signedAngleRadians));
    gp_Vec pullComponent(pull); pullComponent.Multiply(std::sin(definition.signedAngleRadians));
    expected.Add(pullComponent);
    if (expected.Magnitude() <= tolerance) return false;
    const gp_Dir expectedNormal(expected);
    for (TopExp_Explorer explorer(candidate, TopAbs_FACE); explorer.More(); explorer.Next()) {
        const TopoDS_Face face = TopoDS::Face(explorer.Current());
        BRepAdaptor_Surface surface(face);
        if (surface.GetType() != GeomAbs_Plane
            || !pointOnFace(face, point(intent.sampleOnNeutralLocal), tolerance)) continue;
        const gp_Dir normal = outwardPlaneNormal(face, surface);
        if (normal.Dot(expectedNormal) < 1.0 - 1e-9) continue;
        output = face; outputNormal = normal; ++count;
    }
    return count == 1;
}

inline bool analyticSections(const TopoDS_Face& face, const gp_Dir& candidateNormal,
                             const FaceIntent& intent, const Definition& definition,
                             double tolerance, std::vector<AnalyticSection>& evidence,
                             double& measuredAngle) {
    const gp_Dir original = direction(intent.outwardNormalLocal);
    const gp_Dir pull = direction(definition.pullDirectionLocal);
    const double originalToPull = std::abs(original.Dot(pull));
    if (originalToPull > 1e-10) return false;
    measuredAngle = std::atan2(candidateNormal.Dot(pull), candidateNormal.Dot(original));
    if (std::abs(measuredAngle - definition.signedAngleRadians) > 1e-10) return false;

    double minimum = INFINITY, maximum = -INFINITY;
    const gp_Pnt neutral = point(intent.sampleOnNeutralLocal);
    for (TopExp_Explorer explorer(face, TopAbs_VERTEX); explorer.More(); explorer.Next()) {
        const gp_Pnt vertex = BRep_Tool::Pnt(TopoDS::Vertex(explorer.Current()));
        const double offset = gp_Vec(neutral, vertex).Dot(gp_Vec(pull));
        minimum = std::min(minimum, offset); maximum = std::max(maximum, offset);
    }
    if (!std::isfinite(minimum) || !std::isfinite(maximum)
        || maximum - minimum <= tolerance) return false;

    BRepAdaptor_Surface surface(face); const gp_Pln plane = surface.Plane();
    constexpr std::array<double, 3> pins{{0.2, 0.5, 0.8}};
    for (double pin : pins) {
        const double pullOffset = minimum + (maximum - minimum) * pin;
        const double expectedNormalOffset = -std::tan(definition.signedAngleRadians) * pullOffset;
        const double denominator = candidateNormal.Dot(original);
        if (std::abs(denominator) <= 1e-12) return false;
        const double measuredNormalOffset = -candidateNormal.Dot(pull) * pullOffset / denominator;
        gp_Vec pointVector(pull); pointVector.Multiply(pullOffset);
        gp_Vec normalVector(original); normalVector.Multiply(expectedNormalOffset);
        pointVector.Add(normalVector); const gp_Pnt sectionPoint = neutral.Translated(pointVector);
        if (plane.Distance(sectionPoint) > tolerance) return false;
        BRepClass_FaceClassifier classifier(face, sectionPoint, tolerance);
        if (classifier.State() != TopAbs_IN && classifier.State() != TopAbs_ON) return false;
        if (std::abs(expectedNormalOffset - measuredNormalOffset) > tolerance) return false;
        evidence.push_back({intent.identifier, pullOffset,
                            expectedNormalOffset, measuredNormalOffset});
    }
    return true;
}
} // namespace detail

inline BuildResult Build(const TopoDS_Shape& source, const Definition& definition,
                         const std::atomic_bool& cancelled) noexcept {
    BuildResult output;
    const auto decline = [&](Refusal value) {
        BuildResult result; result.refusal = cancelled.load() ? Refusal::Cancelled : value;
        result.evidence = output.evidence; return result;
    };
    try {
        if (cancelled.load()) return decline(Refusal::Cancelled);
        Refusal refusal;
        if (source.IsNull()) return decline(Refusal::InvalidCandidate);
        if (!Validate(definition, refusal)) return decline(refusal);
        const double tolerance = std::max(Precision::Confusion() * 32,
                                          1e-7 / definition.metersPerLocalUnit);
        std::vector<std::uint8_t> sourceBefore, sourceAfter, candidateBytes;
        if (!detail::exactShapeBytes(source, sourceBefore)) return decline(Refusal::InvalidCandidate);

        BRepBuilderAPI_Copy copy(source, Standard_True, Standard_False);
        if (!copy.IsDone() || copy.Shape().IsNull()) return decline(Refusal::InvalidCandidate);
        const TopoDS_Shape detached = copy.Shape();
        std::vector<TopoDS_Face> selected; selected.reserve(definition.faces.size());
        for (const auto& intent : definition.faces) {
            TopoDS_Face face;
            if (!detail::resolve(detached, intent, tolerance, face, refusal)) return decline(refusal);
            if (intent.surface == SurfaceKind::Cylindrical)
                return decline(Refusal::UnsupportedCylindricalFace);
            if (intent.surface == SurfaceKind::Conical)
                return decline(Refusal::UnsupportedConicalFace);
            if (intent.surface != SurfaceKind::Planar) return decline(Refusal::UnsupportedSurface);
            selected.push_back(face);
        }

        const gp_Pln neutral(detail::point(definition.neutralOriginLocal),
                             detail::direction(definition.neutralNormalLocal));
        detail::SectionWitness sourceSection;
        if (!detail::sectionWitness(detached, neutral, tolerance, sourceSection))
            return decline(Refusal::NeutralPlaneChanged);

        BRepOffsetAPI_DraftAngle draft(detached);
        for (std::size_t index = 0; index < selected.size(); ++index) {
            if (cancelled.load()) return decline(Refusal::Cancelled);
            AddObservation observation; observation.face = definition.faces[index].identifier;
            try {
                draft.Add(selected[index], detail::direction(definition.pullDirectionLocal),
                          definition.signedAngleRadians, neutral, Standard_True);
                observation.addDone = draft.AddDone(); observation.status = int(draft.Status());
            } catch (...) {
                output.evidence.adds.push_back(observation);
                return decline(Refusal::AddDoneFailure);
            }
            output.evidence.adds.push_back(observation);
            if (!observation.addDone) {
                try { draft.Remove(selected[index]); } catch (...) {}
                return decline(Refusal::AddDoneFailure);
            }
        }
        try { draft.Build(); } catch (...) { return decline(Refusal::BuildFailure); }
        if (!draft.IsDone() || draft.Shape().IsNull()) return decline(Refusal::BuildFailure);
        for (const auto& face : selected)
            if (draft.ConnectedFaces(face).Size() != 1)
                return decline(Refusal::PropagatedUnselectedFace);
        const TopoDS_Shape candidate = draft.Shape();
        if (!BRepCheck_Analyzer(candidate).IsValid() || !detail::selfIntersectionFree(candidate))
            return decline(Refusal::InvalidCandidate);

        for (TopExp_Explorer explorer(candidate, TopAbs_SOLID); explorer.More(); explorer.Next())
            ++output.evidence.connectedSolids;
        if (candidate.ShapeType() != TopAbs_SOLID || output.evidence.connectedSolids != 1)
            return decline(Refusal::DisconnectedSolid);

        detail::SectionWitness candidateSection;
        if (!detail::sectionWitness(candidate, neutral, tolerance, candidateSection)
            || !detail::sameSection(sourceSection, candidateSection, tolerance))
            return decline(Refusal::NeutralPlaneChanged);
        output.evidence.neutralSectionEdges = candidateSection.edges;
        output.evidence.neutralSectionLengthLocal = candidateSection.length;

        for (const auto& intent : definition.faces) {
            TopoDS_Face draftedFace; gp_Dir normal;
            if (!detail::candidateFace(candidate, intent, definition, tolerance,
                                       draftedFace, normal))
                return decline(Refusal::SignedAngleMismatch);
            double measured = 0;
            if (!detail::analyticSections(draftedFace, normal, intent, definition,
                                          tolerance, output.evidence.sections, measured))
                return decline(Refusal::AnalyticSectionMismatch);
            output.evidence.measuredSignedAngleRadians = measured;
        }

        if (!detail::exactShapeBytes(source, sourceAfter) || sourceAfter != sourceBefore)
            return decline(Refusal::SourceMutation);
        output.evidence.sourceBytesUnchanged = true;
        if (!detail::exactShapeBytes(candidate, candidateBytes) || candidateBytes == sourceBefore
            || candidate.IsSame(source)) return decline(Refusal::CandidateNotSeparated);
        output.evidence.candidateBytesDistinct = true;
        output.refusal = Refusal::None; output.solid = candidate; return output;
    } catch (...) { return decline(Refusal::BuildFailure); }
}

inline BuildResult BuildDeterministically(const TopoDS_Shape& source,
                                          const Definition& definition,
                                          const std::atomic_bool& cancelled) noexcept {
    BuildResult first = Build(source, definition, cancelled);
    if (!first.built()) return first;
    BuildResult second = Build(source, definition, cancelled);
    if (!second.built()) return second;
    std::vector<std::uint8_t> a, b;
    if (!detail::exactShapeBytes(first.solid, a) || !detail::exactShapeBytes(second.solid, b)
        || a != b || first.evidence.adds.size() != second.evidence.adds.size()
        || first.evidence.sections.size() != second.evidence.sections.size()
        || first.evidence.measuredSignedAngleRadians
            != second.evidence.measuredSignedAngleRadians
        || first.evidence.neutralSectionLengthLocal
            != second.evidence.neutralSectionLengthLocal) {
        BuildResult result; result.refusal = Refusal::ReplayMismatch; return result;
    }
    for (std::size_t index = 0; index < first.evidence.sections.size(); ++index) {
        const auto& lhs = first.evidence.sections[index];
        const auto& rhs = second.evidence.sections[index];
        if (lhs.face != rhs.face || lhs.pullOffsetLocal != rhs.pullOffsetLocal
            || lhs.expectedNormalOffsetLocal != rhs.expectedNormalOffsetLocal
            || lhs.measuredNormalOffsetLocal != rhs.measuredNormalOffsetLocal) {
            BuildResult result; result.refusal = Refusal::ReplayMismatch; return result;
        }
    }
    return first;
}

} // namespace core3d::draft_faces
