#if DEBUG

#import <Foundation/Foundation.h>

#include "../OCCTKit/VariableRadiusFilletBuild.hxx"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRep_Builder.hxx>
#include <Standard_Version.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopoDS.hxx>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {
using namespace core3d::variable_radius_fillet;

UUID UUIDValue(std::uint8_t value) {
    UUID result{};
    result.back() = value;
    return result;
}

Digest DigestValue(std::uint8_t value) {
    Digest result{};
    result.back() = value;
    return result;
}

OrientedEdgeAnchor Anchor(double scale) {
    OrientedEdgeAnchor edge;
    edge.identifier = UUIDValue(3);
    edge.pointLocal = {{0, 0, 20 * scale}};
    edge.tangent = {{0, 0, 1}};
    edge.normalA = {{-1, 0, 0}};
    edge.normalB = {{0, -1, 0}};
    edge.startLocal = {{0, 0, 0}};
    edge.endLocal = {{0, 0, 40 * scale}};
    edge.selectorProof = DigestValue(4);
    return edge;
}

Definition LinearDefinition(double unit) {
    const double scale = 0.001 / unit;
    Definition value;
    value.metersPerLocalUnit = unit;
    value.stations = {{{UUIDValue(1), 0, 2 * scale},
                       {UUIDValue(2), 1, 5 * scale}}};
    value.edges.push_back(Anchor(scale));
    return value;
}

MultiStationDefinition StationDefinition(double unit) {
    const double scale = 0.001 / unit;
    MultiStationDefinition value;
    value.metersPerLocalUnit = unit;
    constexpr double radii[] = {2, 3, 4, 6, 8};
    for (std::size_t index = 0; index < 5; ++index) {
        value.stations.push_back({UUIDValue(std::uint8_t(10 + index)),
                                  double(index) / 4, radii[index] * scale});
    }
    OrientedEdgeAnchor edge = Anchor(scale);
    edge.identifier = UUIDValue(20);
    edge.selectorProof = DigestValue(21);
    value.edges.push_back(edge);
    return value;
}

template<class DefinitionType>
bool DefinitionBytes(const DefinitionType& definition,
                     std::vector<std::uint8_t>& bytes) {
    return Encode(definition, bytes);
}

template<class DefinitionType>
struct CaseResult final {
    BuildResult build;
    bool sourceUnchanged = false;
    bool definitionUnchanged = false;
    std::size_t filletEntries = 0;
    DebugBuildObservation debug;
};

template<class DefinitionType, class Builder>
CaseResult<DefinitionType> RunCase(const TopoDS_Shape& source,
                                   const DefinitionType& definition,
                                   Builder builder) {
    CaseResult<DefinitionType> observed;
    std::vector<std::uint8_t> sourceBefore, sourceAfter;
    std::vector<std::uint8_t> definitionBefore, definitionAfter;
    if (!core3d::variable_radius_fillet::detail::exactShapeBytes(source, sourceBefore)
        || !DefinitionBytes(definition, definitionBefore)) return observed;
    DebugBuildObservation debug;
    DebugBuildObservation* previous = debugBuildObservation;
    debugBuildObservation = &debug;
    observed.build = builder(source, definition);
    debugBuildObservation = previous;
    observed.filletEntries = debug.filletEntryCount;
    observed.debug = std::move(debug);
    observed.sourceUnchanged =
        core3d::variable_radius_fillet::detail::exactShapeBytes(source, sourceAfter)
        && sourceBefore == sourceAfter;
    observed.definitionUnchanged = DefinitionBytes(definition, definitionAfter)
        && definitionBefore == definitionAfter;
    return observed;
}

bool IsCleanRefusal(const BuildResult& value) {
    return !value.built() && value.solid.IsNull()
        && value.evidence.consumedEdges.empty()
        && value.evidence.sections.empty()
        && value.evidence.minimumClearanceLocal == 0
        && value.evidence.removedVolumeLocal3 == 0;
}

bool IsOrientation(const CaseResult<Definition>& value) {
    return IsCleanRefusal(value.build)
        && value.build.refusal == Refusal::OrientationDrift
        && std::strcmp(Reason(value.build.refusal), "b3.orientation-drift") == 0;
}

bool IsOrientation(const CaseResult<MultiStationDefinition>& value) {
    return IsCleanRefusal(value.build)
        && value.build.refusal == Refusal::OrientationDrift
        && std::strcmp(Reason(value.build.refusal), "b3.orientation-drift") == 0;
}

TopoDS_Shape CoincidentBoxes(double scale) {
    const TopoDS_Shape first = BRepPrimAPI_MakeBox(
        30 * scale, 30 * scale, 40 * scale).Shape();
    const TopoDS_Shape second = BRepBuilderAPI_Copy(
        first, Standard_True, Standard_False).Shape();
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    builder.Add(compound, first);
    builder.Add(compound, second);
    return compound;
}

TopoDS_Shape TranslatedBox(double scale) {
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(
        30 * scale, 30 * scale, 40 * scale).Shape();
    gp_Trsf transform;
    transform.SetTranslation(gp_Vec(5 * scale, 0, 0));
    return BRepBuilderAPI_Transform(
        box, transform, Standard_True, Standard_False).Shape();
}

TopoDS_Shape SingleSupportFace(const TopoDS_Shape& box,
                               const OrientedEdgeAnchor& anchor,
                               double tolerance) {
    TopoDS_Edge edge;
    Refusal refusal = Refusal::KernelFailure;
    if (!core3d::variable_radius_fillet::detail::resolve(
            box, anchor, tolerance, edge, refusal)) return {};
    TopTools_IndexedDataMapOfShapeListOfShape owners;
    TopExp::MapShapesAndAncestors(box, TopAbs_EDGE, TopAbs_FACE, owners);
    if (!owners.Contains(edge)) return {};
    TopTools_ListIteratorOfListOfShape iterator(owners.FindFromKey(edge));
    return iterator.More() ? TopoDS::Face(iterator.Value()) : TopoDS_Shape{};
}

bool Correspondence(const TopoDS_Shape& box, const OrientedEdgeAnchor& anchor,
                    double tolerance, bool reverseEnumeration,
                    Refusal& refusal) {
    TopoDS_Edge edge;
    if (!core3d::variable_radius_fillet::detail::resolve(
            box, anchor, tolerance, edge, refusal)) return false;
    double clearance = 0;
    return core3d::variable_radius_fillet::detail::continuousClearance(
        box, edge, anchor, tolerance, clearance, refusal, reverseEnumeration);
}
} // namespace

extern "C" void Core3DDebugB3VariableFilletRepairProbe(
    double unit, std::int32_t* observations, double* scalars) noexcept {
    if (!observations || !scalars) return;
    for (std::size_t index = 0; index < 36; ++index) observations[index] = 0;
    for (std::size_t index = 0; index < 8; ++index) scalars[index] = 0;
    observations[0] = 1;
    try {
        const double scale = 0.001 / unit;
        const double tolerance = std::max(Precision::Confusion() * 32, 1e-7 / unit);
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(
            30 * scale, 30 * scale, 40 * scale).Shape();
        const std::atomic_bool running{false};
        const std::atomic_bool cancelled{true};
        const Definition linear = LinearDefinition(unit);
        const MultiStationDefinition stations = StationDefinition(unit);

        Refusal validation = Refusal::KernelFailure;
        observations[1] = Validate(linear, validation) && validation == Refusal::None;
        observations[2] = Validate(stations, validation) && validation == Refusal::None;

        std::vector<std::uint8_t> linearBytes, linearSecond;
        Definition linearDecoded;
        observations[3] = Encode(linear, linearBytes)
            && Decode(linearBytes, linearDecoded) && linearDecoded == linear
            && Encode(linearDecoded, linearSecond) && linearBytes == linearSecond;
        std::vector<std::uint8_t> stationBytes, stationSecond;
        MultiStationDefinition stationDecoded;
        observations[4] = Encode(stations, stationBytes)
            && Decode(stationBytes, stationDecoded) && stationDecoded == stations
            && Encode(stationDecoded, stationSecond) && stationBytes == stationSecond;
        Definition rejectedLinear;
        MultiStationDefinition rejectedStations;
        observations[5] = !Decode(stationBytes, rejectedLinear)
            && !Decode(linearBytes, rejectedStations);

        const auto positiveLinear = RunCase(box, linear,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return BuildDeterministically(source, definition, running);
            });
        const auto positiveStations = RunCase(box, stations,
            [&](const TopoDS_Shape& source, const MultiStationDefinition& definition) {
                return BuildMultiStationDeterministically(source, definition, running);
            });
        observations[6] = positiveLinear.build.built();
        observations[7] = positiveStations.build.built();
        observations[8] = positiveLinear.build.evidence.consumedEdges.size() == 1
            && positiveLinear.build.evidence.sections.size() == 3
            && positiveLinear.build.evidence.removedVolumeLocal3 > 0
            && positiveStations.build.evidence.consumedEdges.size() == 1
            && positiveStations.build.evidence.sections.size() == 3
            && positiveStations.build.evidence.removedVolumeLocal3 > 0;
        observations[9] = positiveLinear.sourceUnchanged
            && positiveLinear.definitionUnchanged
            && positiveStations.sourceUnchanged
            && positiveStations.definitionUnchanged;
        observations[10] = std::int32_t(positiveLinear.filletEntries
                                        + positiveStations.filletEntries);

        Definition linearBoth = linear;
        linearBoth.edges[0].normalA = {{1, 0, 0}};
        linearBoth.edges[0].normalB = {{0, 1, 0}};
        Definition linearA = linear;
        linearA.edges[0].normalA = {{1, 0, 0}};
        Definition linearB = linear;
        linearB.edges[0].normalB = {{0, 1, 0}};
        MultiStationDefinition stationBoth = stations;
        stationBoth.edges[0].normalA = {{1, 0, 0}};
        stationBoth.edges[0].normalB = {{0, 1, 0}};
        MultiStationDefinition stationA = stations;
        stationA.edges[0].normalA = {{1, 0, 0}};
        MultiStationDefinition stationB = stations;
        stationB.edges[0].normalB = {{0, 1, 0}};
        observations[11] = Validate(linearBoth, validation)
            && Validate(linearA, validation) && Validate(linearB, validation)
            && Validate(stationBoth, validation) && Validate(stationA, validation)
            && Validate(stationB, validation);

        const auto rejectedLinearBoth = RunCase(box, linearBoth,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return Build(source, definition, running);
            });
        const auto rejectedLinearA = RunCase(box, linearA,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return Build(source, definition, running);
            });
        const auto rejectedLinearB = RunCase(box, linearB,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return Build(source, definition, running);
            });
        const auto rejectedStationBoth = RunCase(box, stationBoth,
            [&](const TopoDS_Shape& source, const MultiStationDefinition& definition) {
                return BuildMultiStation(source, definition, running);
            });
        const auto rejectedStationA = RunCase(box, stationA,
            [&](const TopoDS_Shape& source, const MultiStationDefinition& definition) {
                return BuildMultiStation(source, definition, running);
            });
        const auto rejectedStationB = RunCase(box, stationB,
            [&](const TopoDS_Shape& source, const MultiStationDefinition& definition) {
                return BuildMultiStation(source, definition, running);
            });
        observations[12] = IsOrientation(rejectedLinearBoth);
        observations[13] = IsOrientation(rejectedLinearA);
        observations[14] = IsOrientation(rejectedLinearB);
        observations[15] = IsOrientation(rejectedStationBoth);
        observations[16] = IsOrientation(rejectedStationA);
        observations[17] = IsOrientation(rejectedStationB);
        observations[18] = rejectedLinearBoth.sourceUnchanged
            && rejectedLinearBoth.definitionUnchanged
            && rejectedLinearA.sourceUnchanged && rejectedLinearA.definitionUnchanged
            && rejectedLinearB.sourceUnchanged && rejectedLinearB.definitionUnchanged
            && rejectedStationBoth.sourceUnchanged
            && rejectedStationBoth.definitionUnchanged
            && rejectedStationA.sourceUnchanged && rejectedStationA.definitionUnchanged
            && rejectedStationB.sourceUnchanged && rejectedStationB.definitionUnchanged;
        observations[19] = std::int32_t(rejectedLinearBoth.filletEntries
            + rejectedLinearA.filletEntries + rejectedLinearB.filletEntries
            + rejectedStationBoth.filletEntries + rejectedStationA.filletEntries
            + rejectedStationB.filletEntries);

        const auto ambiguous = RunCase(CoincidentBoxes(scale), linear,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return Build(source, definition, running);
            });
        observations[20] = IsCleanRefusal(ambiguous.build)
            && ambiguous.build.refusal == Refusal::AnchorAmbiguous;
        const auto missing = RunCase(TranslatedBox(scale), linear,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return Build(source, definition, running);
            });
        observations[21] = IsCleanRefusal(missing.build)
            && missing.build.refusal == Refusal::AnchorMissing;
        const TopoDS_Shape oneFace = SingleSupportFace(box, linear.edges[0], tolerance);
        const auto insufficient = RunCase(oneFace, linear,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return Build(source, definition, running);
            });
        observations[22] = IsCleanRefusal(insufficient.build)
            && insufficient.build.refusal == Refusal::Clearance;
        const auto stoppedLinear = RunCase(box, linear,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return Build(source, definition, cancelled);
            });
        const auto stoppedStations = RunCase(box, stations,
            [&](const TopoDS_Shape& source, const MultiStationDefinition& definition) {
                return BuildMultiStation(source, definition, cancelled);
            });
        observations[23] = IsCleanRefusal(stoppedLinear.build)
            && stoppedLinear.build.refusal == Refusal::Cancelled
            && stoppedLinear.filletEntries == 0
            && stoppedLinear.sourceUnchanged && stoppedLinear.definitionUnchanged;
        observations[24] = IsCleanRefusal(stoppedStations.build)
            && stoppedStations.build.refusal == Refusal::Cancelled
            && stoppedStations.filletEntries == 0
            && stoppedStations.sourceUnchanged && stoppedStations.definitionUnchanged;

        Refusal direct = Refusal::KernelFailure;
        Refusal reversed = Refusal::KernelFailure;
        Refusal flippedDirect = Refusal::KernelFailure;
        Refusal flippedReversed = Refusal::KernelFailure;
        observations[25] = Correspondence(
            box, linear.edges[0], tolerance, false, direct)
            && direct == Refusal::None;
        observations[26] = Correspondence(
            box, linear.edges[0], tolerance, true, reversed)
            && reversed == Refusal::None;
        observations[27] = !Correspondence(
            box, linearBoth.edges[0], tolerance, false, flippedDirect)
            && flippedDirect == Refusal::OrientationDrift;
        observations[28] = !Correspondence(
            box, linearBoth.edges[0], tolerance, true, flippedReversed)
            && flippedReversed == Refusal::OrientationDrift;
        observations[29] = std::strcmp(Reason(Refusal::OrientationDrift),
                                       "b3.orientation-drift") == 0;

        const TopoDS_Shape sourceDrift = BRepPrimAPI_MakeBox(
            30 * scale, 30 * scale, 50 * scale).Shape();
        const auto drift = RunCase(sourceDrift, linear,
            [&](const TopoDS_Shape& source, const Definition& definition) {
                return Build(source, definition, running);
            });
        observations[31] = IsCleanRefusal(drift.build)
            && drift.build.refusal == Refusal::OrientationDrift
            && drift.filletEntries == 0
            && drift.sourceUnchanged && drift.definitionUnchanged;

        scalars[0] = core3d::variable_radius_fillet::detail::dot(
            linear.edges[0].normalA, linear.edges[0].normalA);
        scalars[1] = core3d::variable_radius_fillet::detail::dot(
            linear.edges[0].normalA, linear.edges[0].normalB);
        scalars[2] = core3d::variable_radius_fillet::detail::dot(
            linear.edges[0].normalB, linear.edges[0].normalA);
        scalars[3] = core3d::variable_radius_fillet::detail::dot(
            linear.edges[0].normalB, linear.edges[0].normalB);
        scalars[4] = core3d::variable_radius_fillet::detail::dot(
            linear.edges[0].normalA, linearBoth.edges[0].normalA);
        scalars[5] = core3d::variable_radius_fillet::detail::dot(
            linear.edges[0].normalA, linearBoth.edges[0].normalB);
        scalars[6] = core3d::variable_radius_fillet::detail::dot(
            linear.edges[0].normalB, linearBoth.edges[0].normalA);
        scalars[7] = core3d::variable_radius_fillet::detail::dot(
            linear.edges[0].normalB, linearBoth.edges[0].normalB);
        observations[0] = 100;
    } catch (...) {
        observations[30] = 1;
        observations[0] = -1;
        debugBuildObservation = nullptr;
    }
}

namespace {
NSData *DataValue(const std::vector<std::uint8_t>& bytes) {
    return [NSData dataWithBytes:bytes.data() length:bytes.size()];
}

template<std::size_t Count>
NSArray<NSNumber *> *Numbers(const std::array<double, Count>& values) {
    NSMutableArray<NSNumber *> *result = [NSMutableArray arrayWithCapacity:Count];
    for (double value : values) [result addObject:@(value)];
    return result;
}

NSArray<NSNumber *> *Numbers(const std::vector<double>& values) {
    NSMutableArray<NSNumber *> *result = [NSMutableArray arrayWithCapacity:values.size()];
    for (double value : values) [result addObject:@(value)];
    return result;
}

NSArray<NSNumber *> *Integers(const std::vector<int>& values) {
    NSMutableArray<NSNumber *> *result = [NSMutableArray arrayWithCapacity:values.size()];
    for (int value : values) [result addObject:@(value)];
    return result;
}

NSDictionary *BuildObservation(const BuildResult& result) {
    return @{
        @"raw": @(std::uint8_t(result.refusal)),
        @"reason": [NSString stringWithUTF8String:Reason(result.refusal)],
        @"solidNull": @(result.solid.IsNull()),
        @"consumedCount": @(result.evidence.consumedEdges.size()),
        @"sectionCount": @(result.evidence.sections.size()),
        @"removedVolumeLocal3": @(result.evidence.removedVolumeLocal3),
        @"minimumClearanceLocal": @(result.evidence.minimumClearanceLocal)
    };
}

template<class DefinitionType>
NSDictionary *CaseObservation(const CaseResult<DefinitionType>& result) {
    return @{
        @"build": BuildObservation(result.build),
        @"sourceUnchanged": @(result.sourceUnchanged),
        @"definitionUnchanged": @(result.definitionUnchanged),
        @"filletEntries": @(result.filletEntries)
    };
}

NSDictionary *LawObservation(const DebugRealizedLawObservation& law) {
    NSMutableArray<NSDictionary *> *pieces = [NSMutableArray array];
    for (const auto& piece : law.pieces) {
        [pieces addObject:@{
            @"type": [NSString stringWithUTF8String:piece.type.c_str()],
            @"first": @(piece.first),
            @"last": @(piece.last),
            @"continuity": @(piece.continuity),
            @"degree": @(piece.degree),
            @"rational": @(piece.rational),
            @"c1Intervals": Numbers(piece.c1Intervals),
            @"knots": Numbers(piece.knots),
            @"multiplicities": Integers(piece.multiplicities),
            @"poles": Numbers(piece.poles)
        }];
    }
    NSMutableArray<NSDictionary *> *spans = [NSMutableArray array];
    for (const auto& span : law.spans) {
        [spans addObject:@{
            @"piece": @(span.piece),
            @"first": @(span.first),
            @"last": @(span.last),
            @"derivativeFirst": @(span.derivativeFirst),
            @"derivativeLast": @(span.derivativeLast),
            @"extremumParameter": @(span.extremumParameter),
            @"extremumDerivative": @(span.extremumDerivative),
            @"minimumDerivative": @(span.minimumDerivative),
            @"maximumDerivative": @(span.maximumDerivative)
        }];
    }
    NSMutableArray<NSDictionary *> *joins = [NSMutableArray array];
    for (const auto& join : law.joins) {
        [joins addObject:@{
            @"parameter": @(join.parameter),
            @"valueLeft": @(join.valueLeft),
            @"valueRight": @(join.valueRight),
            @"derivativeLeft": @(join.derivativeLeft),
            @"derivativeRight": @(join.derivativeRight)
        }];
    }
    return @{
        @"attempted": @(law.attempted),
        @"exception": @(law.exception),
        @"supported": @(law.supported),
        @"c1OnPhysicalInterval": @(law.c1OnPhysicalInterval),
        @"nonnegativeDerivative": @(law.nonnegativeDerivative),
        @"noConstantSpan": @(law.noConstantSpan),
        @"physicalBounds": @[@(law.physicalFirst), @(law.physicalLast)],
        @"reportedBounds": @[@(law.boundsFirst), @(law.boundsLast)],
        @"stationParameters": Numbers(law.stationParameters),
        @"stationRadii": Numbers(law.stationRadii),
        @"sampleParameters": Numbers(law.sampleParameters),
        @"samplePositionsZ": Numbers(law.samplePositionsZ),
        @"sampleValues": Numbers(law.sampleValues),
        @"sampleDerivatives": Numbers(law.sampleDerivatives),
        @"gridValues": Numbers(law.gridValues),
        @"gridDerivatives": Numbers(law.gridDerivatives),
        @"pieces": pieces,
        @"spans": spans,
        @"joins": joins
    };
}

std::vector<std::uint8_t> ShapeBytes(const TopoDS_Shape& shape) {
    std::vector<std::uint8_t> result;
    core3d::variable_radius_fillet::detail::exactShapeBytes(shape, result);
    return result;
}

NSDictionary *ObserveRealizedLaw(double unit) {
    const double scale = 0.001 / unit;
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(
        30 * scale, 30 * scale, 40 * scale).Shape();
    const Definition definition = LinearDefinition(unit);
    const std::atomic_bool running{false};
    const std::atomic_bool cancelledFlag{true};

    const auto sourceBefore = ShapeBytes(box);
    std::vector<std::uint8_t> definitionBefore;
    const bool definitionEncoded = Encode(definition, definitionBefore);
    const auto positiveA = RunCase(box, definition,
        [&](const TopoDS_Shape& source, const Definition& value) {
            return BuildDeterministically(source, value, running);
        });
    const auto positiveB = RunCase(box, definition,
        [&](const TopoDS_Shape& source, const Definition& value) {
            return BuildDeterministically(source, value, running);
        });
    const auto sourceAfterPositive = ShapeBytes(box);
    std::vector<std::uint8_t> definitionAfterPositive;
    Encode(definition, definitionAfterPositive);
    const auto candidateA = ShapeBytes(positiveA.build.solid);
    const auto candidateB = ShapeBytes(positiveB.build.solid);

    NSMutableArray<NSDictionary *> *laws = [NSMutableArray array];
    for (const auto& law : positiveA.debug.realizedLaws)
        [laws addObject:LawObservation(law)];
    for (const auto& law : positiveB.debug.realizedLaws)
        [laws addObject:LawObservation(law)];

    NSMutableArray<NSNumber *> *sectionParameters = [NSMutableArray array];
    NSMutableArray<NSNumber *> *sectionExpected = [NSMutableArray array];
    NSMutableArray<NSNumber *> *sectionMeasured = [NSMutableArray array];
    for (const auto& section : positiveA.build.evidence.sections) {
        [sectionParameters addObject:@(section.parameter)];
        [sectionExpected addObject:@(section.expectedRadiusLocal)];
        [sectionMeasured addObject:@(section.measuredRadiusLocal)];
    }

    const auto cancelled = RunCase(box, definition,
        [&](const TopoDS_Shape& source, const Definition& value) {
            return Build(source, value, cancelledFlag);
        });
    const TopoDS_Shape tight = BRepPrimAPI_MakeBox(
        6 * scale, 6 * scale, 40 * scale).Shape();
    const auto clearance = RunCase(tight, definition,
        [&](const TopoDS_Shape& source, const Definition& value) {
            return Build(source, value, running);
        });
    Definition flipped = definition;
    flipped.edges[0].normalA = {{1, 0, 0}};
    flipped.edges[0].normalB = {{0, 1, 0}};
    const auto orientation = RunCase(box, flipped,
        [&](const TopoDS_Shape& source, const Definition& value) {
            return Build(source, value, running);
        });

    Definition decoded;
    std::vector<std::uint8_t> definitionReencoded;
    const bool definitionDecoded = Decode(definitionBefore, decoded)
        && Encode(decoded, definitionReencoded);
    const MultiStationDefinition stationDefinition = StationDefinition(unit);
    std::vector<std::uint8_t> stationBytes, stationReencoded;
    const bool stationEncoded = Encode(stationDefinition, stationBytes);
    MultiStationDefinition stationDecoded;
    const bool stationDecodedOK = Decode(stationBytes, stationDecoded)
        && Encode(stationDecoded, stationReencoded);
    Definition rejectsStations;
    MultiStationDefinition rejectsDefinition;

    Definition equal = definition;
    equal.stations[1].radiusLocal = equal.stations[0].radiusLocal;
    Refusal equalRefusal = Refusal::KernelFailure;
    const bool equalAccepted = Validate(equal, equalRefusal);
    Definition closed = definition;
    closed.edges[0].curve = CurveKind(2);
    Refusal closedRefusal = Refusal::None;
    const bool closedAccepted = Validate(closed, closedRefusal);
    Definition budget = definition;
    budget.edges.clear();
    for (std::size_t index = 0; index <= MaximumEdges; ++index) {
        OrientedEdgeAnchor edge = Anchor(scale);
        edge.identifier = UUIDValue(std::uint8_t(30 + index));
        edge.selectorProof = DigestValue(std::uint8_t(60 + index));
        budget.edges.push_back(edge);
    }
    Refusal budgetRefusal = Refusal::None;
    const bool budgetAccepted = Validate(budget, budgetRefusal);

    return @{
        @"bridgeVersion": @2,
        @"linkedOCCT": @OCC_VERSION_COMPLETE,
        @"metersPerLocalUnit": @(unit),
        @"scale": @(scale),
        @"positiveA": CaseObservation(positiveA),
        @"positiveB": CaseObservation(positiveB),
        @"positiveValid": @(!positiveA.build.solid.IsNull()
            && BRepCheck_Analyzer(positiveA.build.solid).IsValid()),
        @"expectedEdgeConsumed": @(positiveA.build.evidence.consumedEdges
            == std::vector<UUID>{UUIDValue(3)}),
        @"sourceBefore": DataValue(sourceBefore),
        @"sourceAfterPositive": DataValue(sourceAfterPositive),
        @"candidateA": DataValue(candidateA),
        @"candidateB": DataValue(candidateB),
        @"definitionEncoded": @(definitionEncoded),
        @"definitionBefore": DataValue(definitionBefore),
        @"definitionAfterPositive": DataValue(definitionAfterPositive),
        @"lawInvocationCount": @(laws.count),
        @"lawInvocations": laws,
        @"sectionParameters": sectionParameters,
        @"sectionExpected": sectionExpected,
        @"sectionMeasured": sectionMeasured,
        @"cancelled": CaseObservation(cancelled),
        @"clearance": CaseObservation(clearance),
        @"orientation": CaseObservation(orientation),
        @"definitionDecoded": @(definitionDecoded),
        @"definitionReencoded": DataValue(definitionReencoded),
        @"stationEncoded": @(stationEncoded),
        @"stationDecoded": @(stationDecodedOK),
        @"stationBytes": DataValue(stationBytes),
        @"stationReencoded": DataValue(stationReencoded),
        @"definitionRejectsStations": @(!Decode(stationBytes, rejectsStations)),
        @"stationsRejectDefinition": @(!Decode(definitionBefore, rejectsDefinition)),
        @"equalAccepted": @(equalAccepted),
        @"equalRefusal": @(std::uint8_t(equalRefusal)),
        @"closedAccepted": @(closedAccepted),
        @"closedRefusal": @(std::uint8_t(closedRefusal)),
        @"closedReason": [NSString stringWithUTF8String:Reason(closedRefusal)],
        @"budgetAccepted": @(budgetAccepted),
        @"budgetRefusal": @(std::uint8_t(budgetRefusal)),
        @"budgetReason": [NSString stringWithUTF8String:Reason(budgetRefusal)]
    };
}
} // namespace

extern "C" void *Core3DDebugB3F01RealizedLawObserve(double unit) {
    @autoreleasepool {
        try {
            return (__bridge_retained void *)ObserveRealizedLaw(unit);
        } catch (...) {
            NSDictionary *failure = @{
                @"bridgeVersion": @2,
                @"bridgeException": @YES,
                @"metersPerLocalUnit": @(unit)
            };
            return (__bridge_retained void *)failure;
        }
    }
}

#endif
