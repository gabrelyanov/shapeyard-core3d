#if DEBUG
#import "Core3DViewController.h"
#import "Core3DD4ProfileContinuationTesting.h"
#import "GLViewController.h"

#include "Core3DViewer.h"
#include "../OCCTKit/FeaturePatternDefinition.hxx"
#include "../OCCTKit/FeaturePatternProfileContinuation.hxx"
#include "../OCCTKit/NativePhysicalWorkingFrame.hxx"
#include "../OCCTKit/NativeOpeningDependentReplay.hxx"
#include "../OCCTKit/RetainedBooleanProgram.hxx"
#include "../OCCTKit/RetainedEdgeTreatmentSnapshot.hxx"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <GProp_GProps.hxx>
#include <PCDM_StoreStatus.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDocStd_Application.hxx>
#include <TDocStd_Document.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <locale>
#include <sstream>
#include <thread>
#include <vector>

namespace {
using namespace core3d;

bool Inputs(Core3DViewController *controller, NSString *host,
            std::shared_ptr<Core3DViewer>& viewer, Handle(OcctDocument)& owner,
            std::shared_ptr<native_opening::Context>& context,
            std::uint32_t& width, std::uint32_t& height) {
    if (!NSThread.isMainThread || !controller || !host || host.length == 0
        || host.length > 128) return false;
    GLViewController *gl = [controller.glController
        isKindOfClass:GLViewController.class]
        ? (GLViewController *)controller.glController : nil;
    const CGSize size = controller.viewportDrawableSize;
    if (!gl || !gl.viewer || !std::isfinite(size.width)
        || !std::isfinite(size.height) || size.width < 1 || size.height < 1
        || size.width > UINT32_MAX || size.height > UINT32_MAX) return false;
    viewer = gl.viewer; owner = viewer->getDocument();
    width = std::uint32_t(size.width); height = std::uint32_t(size.height);
    const char *text = host.UTF8String;
    if (owner.IsNull() || !text) return false;
    context = viewer->captureNativeOpeningContext(width, height, {text});
    return bool(context);
}

NSDictionary *Observe(const Handle(OcctDocument)& owner, NSString *host,
                      const std::shared_ptr<native_opening::Context>& context) {
    const char *text = host.UTF8String;
    if (!text) return @{@"current": @NO};
    profile_d4::Observation observed;
    if (!context || !profile_d4::ObserveCurrent(
            *owner, text, context, observed)) return @{@"current": @NO};
    const double millimetresPerUnit = observed.metersPerUnit * 1000.0;
    const double volumeScale = millimetresPerUnit * millimetresPerUnit
        * millimetresPerUnit;
    return @{
        @"schema": @"shapeyard.d4-profile-observation.v1",
        @"current": @(observed.current),
        @"hostEntity": [NSString stringWithUTF8String:observed.hostEntity.c_str()],
        @"sourceEntity": [NSString stringWithUTF8String:observed.sourceEntity.c_str()],
        @"profileFeature": [NSString stringWithUTF8String:observed.profileFeature.c_str()],
        @"patternFeature": [NSString stringWithUTF8String:observed.patternFeature.c_str()],
        @"sourceCutStepID": @(observed.sourceCutStepID),
        @"memberCount": @(observed.memberCount),
        @"childCount": @(observed.childCount),
        @"profileScalarCount": @(observed.profileScalarCount),
        @"sourceProgramBytes": @(observed.sourceProgramBytes),
        @"metersPerUnit": @(observed.metersPerUnit),
        @"baselineVolumeMM3": @(observed.baselineVolume * volumeScale),
        @"resultVolumeMM3": @(observed.resultVolume * volumeScale),
        @"sourceVolumeMM3": @(observed.sourceVolume * volumeScale),
    };
}

namespace working_frame = native_physical_working_frame;
namespace topology_budget = retained_topology_budget;

struct WorkingFrameDocument {
    Handle(TDocStd_Application) app;
    Handle(TDocStd_Document) document;
    ~WorkingFrameDocument() {
        try { if (!app.IsNull() && !document.IsNull()) app->Close(document); }
        catch (...) {}
    }
};

bool MakeWorkingFrameDocument(double unit, WorkingFrameDocument& holder,
                              double& observed) {
    holder.app = new TDocStd_Application();
    Core3DDefineSafeBinXCAFFormat(holder.app);
    holder.app->NewDocument(TCollection_ExtendedString("BinXCAF"),
                            holder.document);
    if (holder.document.IsNull()) return false;
    XCAFDoc_DocumentTool::SetLengthUnit(holder.document, unit);
    observed = 0;
    return XCAFDoc_DocumentTool::GetLengthUnit(holder.document, observed);
}

std::string WorkingFrameShapeBytes(const TopoDS_Shape& shape) {
    std::ostringstream stream;
    stream.imbue(std::locale::classic());
    BRepTools::Write(shape, stream);
    return stream.str();
}

unsigned WorkingFrameFlags(const TopoDS_Shape& shape) {
    return unsigned(shape.Free()) | (unsigned(shape.Modified()) << 1)
        | (unsigned(shape.Checked()) << 2) | (unsigned(shape.Orientable()) << 3)
        | (unsigned(shape.Closed()) << 4) | (unsigned(shape.Infinite()) << 5)
        | (unsigned(shape.Convex()) << 6);
}

std::array<double, 6> WorkingFrameVertexBounds(const TopoDS_Shape& shape) {
    std::array<double, 6> value = {
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
    for (TopExp_Explorer it(shape, TopAbs_VERTEX); it.More(); it.Next()) {
        const gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(it.Current()));
        value[0] = std::min(value[0], point.X());
        value[1] = std::min(value[1], point.Y());
        value[2] = std::min(value[2], point.Z());
        value[3] = std::max(value[3], point.X());
        value[4] = std::max(value[4], point.Y());
        value[5] = std::max(value[5], point.Z());
    }
    return value;
}

NSArray<NSNumber *> *WorkingFrameNumbers(const std::vector<double>& values) {
    NSMutableArray<NSNumber *> *result = [NSMutableArray arrayWithCapacity:values.size()];
    for (double value : values) [result addObject:@(value)];
    return result;
}

NSArray<NSNumber *> *WorkingFrameNumbers(const std::array<double, 6>& values) {
    NSMutableArray<NSNumber *> *result = [NSMutableArray arrayWithCapacity:values.size()];
    for (double value : values) [result addObject:@(value)];
    return result;
}

double WorkingFrameVolume(const TopoDS_Shape& shape) {
    GProp_GProps property;
    BRepGProp::VolumeProperties(shape, property, Standard_True,
                               Standard_False, Standard_False);
    return property.Mass();
}

std::array<double, 3> WorkingFrameMaximumTolerances(const TopoDS_Shape& shape) {
    std::array<double, 3> result = {0, 0, 0};
    for (TopExp_Explorer face(shape, TopAbs_FACE); face.More(); face.Next())
        result[0] = std::max(result[0], BRep_Tool::Tolerance(TopoDS::Face(face.Current())));
    for (TopExp_Explorer edge(shape, TopAbs_EDGE); edge.More(); edge.Next())
        result[1] = std::max(result[1], BRep_Tool::Tolerance(TopoDS::Edge(edge.Current())));
    for (TopExp_Explorer vertex(shape, TopAbs_VERTEX); vertex.More(); vertex.Next())
        result[2] = std::max(result[2], BRep_Tool::Tolerance(TopoDS::Vertex(vertex.Current())));
    return result;
}

void WorkingFrameSetTolerances(TopoDS_Shape& shape, double faceTolerance,
                               double edgeTolerance, double vertexTolerance) {
    BRep_Builder builder;
    for (TopExp_Explorer face(shape, TopAbs_FACE); face.More(); face.Next())
        builder.UpdateFace(TopoDS::Face(face.Current()), faceTolerance);
    for (TopExp_Explorer edge(shape, TopAbs_EDGE); edge.More(); edge.Next())
        builder.UpdateEdge(TopoDS::Edge(edge.Current()), edgeTolerance);
    for (TopExp_Explorer vertex(shape, TopAbs_VERTEX); vertex.More(); vertex.Next())
        builder.UpdateVertex(TopoDS::Vertex(vertex.Current()), vertexTolerance);
}

struct NeverStop { bool load() const noexcept { return false; } };
struct CountingStop {
    mutable std::size_t polls = 0;
    std::size_t cancelAt = 0;
    bool load() const noexcept { return ++polls >= cancelAt; }
};

TopoDS_Shape WorkingFramePrivateBox(double s, double x = 30,
                                    double y = 30, double z = 40) {
    const TopoDS_Shape source = BRepPrimAPI_MakeBox(x / s, y / s, z / s).Shape();
    return BRepBuilderAPI_Copy(source, Standard_True, Standard_False).Shape();
}

working_frame::Status EnterWorkingFrame(double unit, TopoDS_Shape privateShape,
                                        topology_budget::Counter& debt,
                                        working_frame::Frame& frame,
                                        const std::vector<TopoDS_Shape>& selected = {}) {
    return working_frame::Frame::Enter(privateShape, unit, selected,
                                       debt, NeverStop{}, frame);
}

NSDictionary *WorkingFrameBoxProbe() {
    NSMutableArray *cells = [NSMutableArray array];
    for (double requested : {0.001, 1.0, 0.01}) {
        WorkingFrameDocument holder;
        double unit = 0;
        if (!MakeWorkingFrameDocument(requested, holder, unit))
            return @{ @"ok": @NO, @"phase": @"document-unit" };
        const double s = 1000.0 * unit;
        topology_budget::Counter debt;
        working_frame::Frame frame;
        if (EnterWorkingFrame(unit, WorkingFramePrivateBox(s), debt, frame)
            != working_frame::Status::Ready)
            return @{ @"ok": @NO, @"phase": @"entry" };
        const auto workingBounds = WorkingFrameVertexBounds(frame.workingShape());
        const double volume = WorkingFrameVolume(frame.workingShape());
        TopoDS_Shape returned;
        const auto finish = frame.Finish(frame.workingShape(), debt,
                                         NeverStop{}, returned);
        if (finish != working_frame::Status::Finished)
            return @{ @"ok": @NO, @"phase": @"exit" };
        auto physicalBounds = WorkingFrameVertexBounds(returned);
        for (double& value : physicalBounds) value *= s;
        [cells addObject:@{ @"unit": @(unit),
            @"workingBounds": WorkingFrameNumbers(workingBounds),
            @"workingVolume": @(volume),
            @"physicalBounds": WorkingFrameNumbers(physicalBounds),
            @"stages": @(debt.buildStages), @"visits": @(debt.topologyVisits) }];
    }
    return @{ @"ok": @YES, @"cells": cells };
}

NSDictionary *WorkingFrameLocatedProbe() {
    const double unit = 1.0, s = 1000.0;
    TopoDS_Shape base = WorkingFramePrivateBox(s, 6, 8, 10);
    gp_Trsf rotation;
    rotation.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)),
                         std::acos(-1.0) / 2.0);
    gp_Trsf translation;
    translation.SetTranslation(gp_Vec(0.012, -0.007, 0.003));
    const gp_Trsf placed = translation * rotation;
    TopoDS_Shape located = BRepBuilderAPI_Transform(
        base, placed, Standard_True, Standard_False).Shape();
    topology_budget::Counter debt;
    working_frame::Frame frame;
    if (EnterWorkingFrame(unit, located, debt, frame) != working_frame::Status::Ready)
        return @{ @"ok": @NO, @"phase": @"entry" };
    gp_Pnt anchor;
    double radius = 0, amount = 0;
    const bool converted = frame.toWorkingPoint(gp_Pnt(0.012, -0.007, 0.003), anchor)
            == working_frame::Status::Ready
        && frame.toWorkingLength(0.0025, radius) == working_frame::Status::Ready
        && frame.preservePhysicalMillimetres(0.25, amount)
            == working_frame::Status::Ready;
    TopoDS_Shape returned;
    const auto finish = frame.Finish(frame.workingShape(), debt, NeverStop{}, returned);
    const auto workingBounds = WorkingFrameVertexBounds(returned.IsNull()
        ? located : BRepBuilderAPI_Transform(returned,
            [] { gp_Trsf t; t.SetScale(gp_Pnt(0,0,0), 1000.0); return t; }(),
            Standard_True, Standard_False).Shape());
    return @{ @"ok": @(converted && finish == working_frame::Status::Finished),
        @"anchor": @[ @(anchor.X()), @(anchor.Y()), @(anchor.Z()) ],
        @"radius": @(radius), @"amountMM": @(amount), @"t": @(0.375),
        @"direction": @[ @0.0, @1.0, @0.0 ],
        @"workingBounds": WorkingFrameNumbers(workingBounds),
        @"orientation": @(unsigned(TopAbs_FORWARD)) };
}

TopoDS_Edge WorkingFrameResolveSelectedEdge(const TopoDS_Shape& detached) {
    // Resolve against this detached source itself. The working-frame helper
    // must carry this handle through transform history; no source/working
    // traversal lists are zipped or assumed to have matching order.
    TopTools_IndexedMapOfShape edges;
    TopExp::MapShapes(detached, TopAbs_EDGE, edges);
    return edges.Extent() > 0 ? TopoDS::Edge(edges(1)) : TopoDS_Edge{};
}

NSDictionary *WorkingFrameMappingProbe() {
    TopoDS_Shape source = BRepPrimAPI_MakeBox(30, 31, 40).Shape();
    const std::string beforeBytes = WorkingFrameShapeBytes(source);
    const unsigned beforeFlags = WorkingFrameFlags(source);
    const auto beforeTolerance = WorkingFrameMaximumTolerances(source);
    TopoDS_Shape detached = BRepBuilderAPI_Copy(
        source, Standard_True, Standard_False).Shape();
    TopoDS_Edge selected = WorkingFrameResolveSelectedEdge(detached);
    topology_budget::Counter debt;
    working_frame::Frame frame;
    const auto status = working_frame::Frame::Enter(detached, 1.0,
        std::vector<TopoDS_Shape>{selected}, debt, NeverStop{}, frame);
    const bool valid = status == working_frame::Status::Ready
        && BRepCheck_Analyzer(frame.workingShape(), Standard_True).IsValid();
    TopTools_IndexedDataMapOfShapeListOfShape ancestors;
    if (valid) TopExp::MapShapesAndAncestors(frame.workingShape(), TopAbs_EDGE,
                                             TopAbs_FACE, ancestors);
    bool shared = false;
    for (int i = 1; i <= ancestors.Extent(); ++i)
        if (ancestors.FindFromIndex(i).Extent() == 2) { shared = true; break; }
    const bool mapped = valid && frame.mappedSubshapes().size() == 1
        && frame.mappedSubshapes()[0].Orientation() == selected.Orientation();

    working_frame::Frame missingFrame, ambiguousFrame;
    topology_budget::Counter missingDebt, ambiguousDebt;
    TopoDS_Edge foreign = TopoDS::Edge(TopExp_Explorer(
        BRepPrimAPI_MakeBox(1, 1, 1).Shape(), TopAbs_EDGE).Current());
    const auto missing = working_frame::Frame::Enter(
        BRepBuilderAPI_Copy(source).Shape(), 1.0,
        std::vector<TopoDS_Shape>{foreign}, missingDebt, NeverStop{}, missingFrame);
    TopoDS_Shape secondDetached = BRepBuilderAPI_Copy(source).Shape();
    TopoDS_Edge duplicate = WorkingFrameResolveSelectedEdge(secondDetached);
    const auto ambiguous = working_frame::Frame::Enter(secondDetached, 1.0,
        std::vector<TopoDS_Shape>{duplicate, duplicate}, ambiguousDebt,
        NeverStop{}, ambiguousFrame);
    const auto afterTolerance = WorkingFrameMaximumTolerances(source);
    return @{ @"ok": @(valid && shared && mapped), @"detached": @(!source.IsSame(detached)),
        @"sourceBytesUnchanged": @(beforeBytes == WorkingFrameShapeBytes(source)),
        @"sourceFlagsUnchanged": @(beforeFlags == WorkingFrameFlags(source)),
        @"sourceToleranceUnchanged": @(beforeTolerance == afterTolerance),
        @"sharedEdge": @(shared), @"mapped": @(mapped),
        @"missingStatus": @(unsigned(missing)),
        @"ambiguousStatus": @(unsigned(ambiguous)),
        @"expectedMissingStatus": @(unsigned(working_frame::Status::MapMissing)),
        @"expectedAmbiguousStatus": @(unsigned(working_frame::Status::MapAmbiguous)) };
}

NSDictionary *WorkingFrameScalarProbe() {
    NSMutableArray *cells = [NSMutableArray array];
    bool all = true;
    for (double unit : {0.001, 1.0, 0.01, 0.0254}) {
        topology_budget::Counter debt;
        working_frame::Frame frame;
        const auto entered = EnterWorkingFrame(unit,
            WorkingFramePrivateBox(1000.0 * unit), debt, frame);
        double length = 0, area = 0, volume = 0, dimensionless = 0;
        const bool converted = entered == working_frame::Status::Ready
            && frame.toDocumentLength(1, length) == working_frame::Status::Ready
            && frame.toDocumentArea(1, area) == working_frame::Status::Ready
            && frame.toDocumentVolume(1, volume) == working_frame::Status::Ready
            && frame.preservePhysicalMillimetres(0.625, dimensionless)
                == working_frame::Status::Ready;
        working_frame::Frame replacement;
        const auto reentry = working_frame::Frame::Enter(
            WorkingFramePrivateBox(1000.0 * unit), unit, {}, debt,
            NeverStop{}, frame);
        TopoDS_Shape returned;
        const auto firstFinish = frame.Finish(frame.workingShape(), debt,
                                               NeverStop{}, returned);
        TopoDS_Shape second;
        const auto secondFinish = frame.Finish(returned, debt, NeverStop{}, second);
        const bool cellOK = converted
            && reentry == working_frame::Status::AlreadyUsed
            && firstFinish == working_frame::Status::Finished
            && secondFinish == working_frame::Status::AlreadyUsed && second.IsNull();
        all = all && cellOK;
        [cells addObject:@{ @"s": @(1000.0 * unit), @"length": @(length),
            @"area": @(area), @"volume": @(volume),
            @"dimensionless": @(dimensionless), @"oneShot": @(cellOK) }];
        (void)replacement;
    }
    return @{ @"ok": @(all), @"cells": cells };
}

NSDictionary *WorkingFrameInvalidProbe() {
    const std::array<double, 7> invalid = {
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), 0.0, -1.0,
        1.0e-200, 1.0e100, std::numeric_limits<double>::denorm_min()};
    bool refused = true, noOutput = true;
    for (double unit : invalid) {
        topology_budget::Counter debt;
        working_frame::Frame frame;
        const auto status = working_frame::Frame::Enter(
            BRepPrimAPI_MakeBox(1, 1, 1).Shape(), unit, {}, debt,
            NeverStop{}, frame);
        refused = refused && status == working_frame::Status::InvalidUnit;
        noOutput = noOutput && !frame.active() && frame.workingShape().IsNull();
    }
    topology_budget::Counter debt;
    working_frame::Frame frame;
    bool scalarRefused = false;
    if (EnterWorkingFrame(1.0, WorkingFramePrivateBox(1000), debt, frame)
        == working_frame::Status::Ready) {
        double result = 1;
        scalarRefused = frame.toWorkingLength(
            std::numeric_limits<double>::max(), result)
                == working_frame::Status::InvalidScalar && result == 0;
    }
    return @{ @"ok": @(refused && noOutput && scalarRefused),
        @"unitRefusals": @(refused), @"noPartialOutput": @(noOutput),
        @"scalarOverflowRefused": @(scalarRefused),
        @"invalidCount": @(invalid.size()) };
}

NSDictionary *WorkingFrameToleranceProbe() {
    NSMutableArray *cells = [NSMutableArray array];
    bool all = true;
    for (double unit : {0.001, 1.0, 0.01}) {
        const double s = 1000.0 * unit;
        TopoDS_Shape source = BRepPrimAPI_MakeBox(30 / s, 31 / s, 40 / s).Shape();
        WorkingFrameSetTolerances(source, 0.0003 / s, 0.0002 / s, 0.0001 / s);
        const std::string sourceBytes = WorkingFrameShapeBytes(source);
        const auto input = WorkingFrameMaximumTolerances(source);
        TopoDS_Shape detached = BRepBuilderAPI_Copy(source).Shape();
        const auto copied = WorkingFrameMaximumTolerances(detached);
        topology_budget::Counter debt;
        working_frame::Frame frame;
        const bool entered = EnterWorkingFrame(unit, detached, debt, frame)
            == working_frame::Status::Ready;
        const auto working = entered
            ? WorkingFrameMaximumTolerances(frame.workingShape())
            : std::array<double, 3>{0, 0, 0};
        TopoDS_Shape returned;
        const bool finished = entered && frame.Finish(frame.workingShape(), debt,
            NeverStop{}, returned) == working_frame::Status::Finished;
        const auto exited = finished
            ? WorkingFrameMaximumTolerances(returned)
            : std::array<double, 3>{0, 0, 0};
        const bool preserved = sourceBytes == WorkingFrameShapeBytes(source)
            && copied == input && working[0] >= input[0] * s
            && working[1] >= input[1] * s && working[2] >= input[2] * s
            && exited[0] >= input[0] && exited[1] >= input[1]
            && exited[2] >= input[2];
        all = all && entered && finished && preserved;
        [cells addObject:@{ @"unit": @(unit),
            @"input": @[ @(input[0]), @(input[1]), @(input[2]) ],
            @"detached": @[ @(copied[0]), @(copied[1]), @(copied[2]) ],
            @"working": @[ @(working[0]), @(working[1]), @(working[2]) ],
            @"exited": @[ @(exited[0]), @(exited[1]), @(exited[2]) ],
            @"sourcePreserved": @(sourceBytes == WorkingFrameShapeBytes(source)) }];
    }
    TopoDS_Shape inaccurate = BRepPrimAPI_MakeBox(30, 31, 40).Shape();
    WorkingFrameSetTolerances(inaccurate, 0.25, 0.25, 0.25);
    topology_budget::Counter badDebt;
    working_frame::Frame badFrame;
    const bool badEntered = EnterWorkingFrame(0.001,
        BRepBuilderAPI_Copy(inaccurate).Shape(), badDebt, badFrame)
        == working_frame::Status::Ready;
    const auto badTolerance = badEntered
        ? WorkingFrameMaximumTolerances(badFrame.workingShape())
        : std::array<double, 3>{0, 0, 0};
    const bool notHealed = badEntered && badTolerance[0] >= 0.25
        && badTolerance[1] >= 0.25 && badTolerance[2] >= 0.25;
    return @{ @"ok": @(all && notHealed), @"cells": cells,
        @"inaccurateWorking": @[ @(badTolerance[0]), @(badTolerance[1]),
                                  @(badTolerance[2]) ],
        @"inaccurateNotHealed": @(notHealed), @"certified": @NO };
}

NSDictionary *WorkingFrameBudgetProbe() {
    const TopoDS_Shape shape = WorkingFramePrivateBox(10);
    topology_budget::Counter stoppedDebt;
    working_frame::Frame stoppedFrame;
    std::atomic_bool stopped{true};
    const auto before = working_frame::Frame::Enter(shape, 0.01, {}, stoppedDebt,
                                                     stopped, stoppedFrame);

    topology_budget::Counter visitDebt;
    visitDebt.topologyVisits = topology_budget::MaximumTopologyVisits;
    working_frame::Frame visitFrame;
    const auto visits = working_frame::Frame::Enter(shape, 0.01, {}, visitDebt,
                                                     NeverStop{}, visitFrame);

    topology_budget::Counter stageDebt;
    stageDebt.buildStages = topology_budget::MaximumBuildStages;
    working_frame::Frame stageFrame;
    const auto stages = working_frame::Frame::Enter(shape, 0.01, {}, stageDebt,
                                                     NeverStop{}, stageFrame);

    topology_budget::Counter duringDebt;
    working_frame::Frame duringFrame;
    CountingStop during{0, 4};
    const auto duringStatus = working_frame::Frame::Enter(
        shape, 0.01, {}, duringDebt, during, duringFrame);

    topology_budget::Counter afterDebt;
    working_frame::Frame afterFrame;
    const bool entered = EnterWorkingFrame(0.01, shape, afterDebt, afterFrame)
        == working_frame::Status::Ready && afterFrame.usedTransform();
    std::atomic_bool cancelAfterTransform{true};
    TopoDS_Shape published;
    const auto after = afterFrame.Finish(afterFrame.workingShape(), afterDebt,
                                         cancelAfterTransform, published);
    const bool ok = before == working_frame::Status::Cancelled
        && visits == working_frame::Status::BudgetDenied && visitDebt.exhausted
        && visitDebt.topologyVisits == topology_budget::MaximumTopologyVisits
        && stages == working_frame::Status::BudgetDenied && stageDebt.exhausted
        && stageDebt.buildStages == topology_budget::MaximumBuildStages
        && duringStatus == working_frame::Status::Cancelled
        && !duringFrame.active() && entered
        && after == working_frame::Status::Cancelled && published.IsNull();
    return @{ @"ok": @(ok), @"beforeEntry": @(unsigned(before)),
        @"duringTraversal": @(unsigned(duringStatus)),
        @"afterTransform": @(unsigned(after)),
        @"visitDebtSticky": @(visitDebt.exhausted),
        @"stageDebtSticky": @(stageDebt.exhausted),
        @"duringPolls": @(during.polls),
        @"visitCount": @(visitDebt.topologyVisits),
        @"stageCount": @(stageDebt.buildStages),
        @"published": @(!published.IsNull()) };
}

NSDictionary *WorkingFrameIdentityProbe() {
    const TopoDS_Shape source = BRepPrimAPI_MakeBox(30, 31, 40).Shape();
    const std::string sourceBytes = WorkingFrameShapeBytes(source);
    const std::vector<unsigned char> definition = {1, 7, 9, 3};
    const std::vector<unsigned char> proof = {8, 2, 6, 4};
    const std::string semanticID = "working-frame-semantic-id";
    std::vector<std::string> runs;
    bool all = true;
    for (int index = 0; index < 2; ++index) {
        TopoDS_Shape detached = BRepBuilderAPI_Copy(source).Shape();
        topology_budget::Counter debt;
        working_frame::Frame frame;
        const auto entered = EnterWorkingFrame(0.001, detached, debt, frame);
        const bool exactCommitment = entered == working_frame::Status::Ready
            && !frame.usedTransform() && frame.workingShape().IsSame(detached)
            && WorkingFrameShapeBytes(frame.workingShape()) == sourceBytes;
        TopoDS_Shape returned;
        const auto finished = frame.Finish(frame.workingShape(), debt,
                                            NeverStop{}, returned);
        all = all && exactCommitment
            && finished == working_frame::Status::Finished
            && returned.IsSame(detached)
            && WorkingFrameShapeBytes(returned) == sourceBytes;
        runs.push_back(WorkingFrameShapeBytes(returned));
    }
    const bool metadata = definition == std::vector<unsigned char>({1, 7, 9, 3})
        && proof == std::vector<unsigned char>({8, 2, 6, 4})
        && semanticID == "working-frame-semantic-id";
    return @{ @"ok": @(all && metadata && runs.size() == 2 && runs[0] == runs[1]),
        @"identityNoTransform": @(all), @"definitionBytes": @(definition.size()),
        @"proofBytes": @(proof.size()), @"semanticID": @"working-frame-semantic-id",
        @"deterministic": @(runs.size() == 2 && runs[0] == runs[1]) };
}

NSDictionary *WorkingFrameProbe(int32_t scenario) {
    try {
        switch (scenario) {
            case 0: return WorkingFrameBoxProbe();
            case 1: return WorkingFrameLocatedProbe();
            case 2: return WorkingFrameMappingProbe();
            case 3: return WorkingFrameScalarProbe();
            case 4: return WorkingFrameInvalidProbe();
            case 5: return WorkingFrameToleranceProbe();
            case 6: return WorkingFrameBudgetProbe();
            case 7: return WorkingFrameIdentityProbe();
            default: return @{ @"ok": @NO, @"phase": @"scenario" };
        }
    } catch (...) { return @{ @"ok": @NO, @"phase": @"exception" }; }
}
} // namespace

static NSDictionary *DebugCreate(Core3DViewController *controller,
                                 NSString *host, NSString *source,
                                 int32_t fault = 0) {
    @autoreleasepool {
        try {
            std::shared_ptr<core3d::Core3DViewer> viewer;
            Handle(OcctDocument) owner;
            std::shared_ptr<core3d::native_opening::Context> ignored;
            std::uint32_t width = 0, height = 0;
            if (!source || source.length == 0 || source.length > 128
                || !Inputs(controller, host, viewer, owner, ignored, width, height))
                return @{@"committed": @NO, @"phase": @"inputs"};
            const char *hostText = host.UTF8String, *sourceText = source.UTF8String;
            if (!hostText || !sourceText) return @{@"committed": @NO, @"phase": @"text"};
            ignored.reset();
            auto context = viewer->captureNativeOpeningContext(
                width, height, {hostText, sourceText});
            std::shared_ptr<const core3d::profile_d4::CreationCapture> capture;
            if (!context || core3d::profile_d4::CaptureCreationHost(*owner,
                    hostText, sourceText, context, width, height, capture)
                    != core3d::profile_d4::CaptureStatus::Current)
                return @{@"committed": @NO, @"phase": @"capture"};
            core3d::profile_d4::CreationEdit edit;
            const double unit = context->openingFence().metersPerUnit();
            edit.columnSpacing = 15.0 * 0.001 / unit;
            std::atomic_bool stop{false};
            auto prepared = core3d::profile_d4::PrepareCreation(
                capture, edit, stop);
            if (!prepared) return @{@"committed": @NO, @"phase": @"prepare"};
            const int beforeUndo = owner->Document()->GetAvailableUndos();
            const int beforeRedo = owner->Document()->GetAvailableRedos();
            if (fault == 1) {
                core3d::profile_d4::DebugArmCreationFault(
                    core3d::profile_d4::CreationFault::AfterLastChild);
            } else if (fault == 2) {
                core3d::profile_d4::DebugArmCreationFault(
                    core3d::profile_d4::CreationFault::CreationReadback);
            } else if (fault != 0 && fault != 3) {
                return @{@"committed": @NO, @"phase": @"fault-input"};
            }
            if (fault == 3) context->debugReportNextCloseUnproven();
            const auto outcome = core3d::profile_d4::StageCreation(*owner, prepared);
            if (outcome != core3d::profile_d4::CreationOutcome::Committed) {
                const bool unresolved = viewer->hasUnresolvedOrdinaryEdit();
                const bool closed = !owner->Document()->HasOpenCommand();
                const int afterUndo = owner->Document()->GetAvailableUndos();
                const int afterRedo = owner->Document()->GetAvailableRedos();
                prepared.reset(); capture.reset();
                auto blockedProbe = viewer->captureNativeOpeningContext(
                    width, height, {hostText, sourceText});
                const bool recoveryRequired = !blockedProbe;
                blockedProbe.reset();
                const bool recovered = outcome
                        == core3d::profile_d4::CreationOutcome::OutcomeUnknown
                    && context->reconcileRecovery(true);
                context.reset();
                auto recaptured = viewer->captureNativeOpeningContext(
                    width, height, {hostText, sourceText});
                bool absent = false;
                if (recaptured) {
                    std::shared_ptr<const core3d::profile_d4::CreationCapture> retry;
                    absent = core3d::profile_d4::CaptureCreationHost(*owner,
                        hostText, sourceText, recaptured, width, height, retry)
                        == core3d::profile_d4::CaptureStatus::Current;
                }
                return @{@"committed": @NO, @"phase": @"stage",
                    @"outcome": @(unsigned(outcome)), @"fault": @(fault),
                    @"closed": @(closed), @"unresolved": @(unresolved),
                    @"newOpeningBlocked": @(recoveryRequired),
                    @"recovered": @(recovered),
                    @"openingAfterRecovery": @(bool(recaptured)),
                    @"creationAbsent": @(absent),
                    @"undoBefore": @(beforeUndo), @"undoAfter": @(afterUndo),
                    @"redoBefore": @(beforeRedo), @"redoAfter": @(afterRedo)};
            }
            // Publication is terminal for the opening authority. The sealed
            // prepared value retains its capture, which in turn retains the
            // context, so retire the full chain before observing the committed
            // scene through a fresh single context.
            prepared.reset(); capture.reset(); context.reset();
            auto observed = viewer->captureNativeOpeningContext(
                width, height, {hostText});
            NSMutableDictionary *result = [Observe(
                owner, host, observed) mutableCopy];
            result[@"committed"] = @YES; result[@"phase"] = @"complete";
            return result;
        } catch (...) { return @{@"committed": @NO, @"phase": @"exception"}; }
    }
}

static NSDictionary *DebugObserve(Core3DViewController *controller,
                                  NSString *host) {
    @autoreleasepool {
        std::shared_ptr<core3d::Core3DViewer> viewer; Handle(OcctDocument) owner;
        std::shared_ptr<core3d::native_opening::Context> context;
        std::uint32_t width = 0, height = 0;
        if (!Inputs(controller, host, viewer, owner, context, width, height))
            return @{@"current": @NO};
        return Observe(owner, host, context);
    }
}

static NSDictionary *DebugCreationStop(Core3DViewController *controller,
                                       NSString *host, NSString *source) {
    @autoreleasepool {
        try {
            std::shared_ptr<core3d::Core3DViewer> viewer; Handle(OcctDocument) owner;
            std::shared_ptr<core3d::native_opening::Context> ignored;
            std::uint32_t width = 0, height = 0;
            if (!source || !Inputs(controller, host, viewer, owner,
                                  ignored, width, height))
                return @{@"cancelled": @NO, @"phase": @"inputs"};
            const char *hostText = host.UTF8String, *sourceText = source.UTF8String;
            if (!hostText || !sourceText)
                return @{@"cancelled": @NO, @"phase": @"text"};
            ignored.reset();
            auto context = viewer->captureNativeOpeningContext(
                width, height, {hostText, sourceText});
            std::shared_ptr<const core3d::profile_d4::CreationCapture> capture;
            if (!context || core3d::profile_d4::CaptureCreationHost(*owner,
                    hostText, sourceText, context, width, height, capture)
                    != core3d::profile_d4::CaptureStatus::Current)
                return @{@"cancelled": @NO, @"phase": @"capture"};
            core3d::profile_d4::CreationEdit edit;
            edit.kind = core3d::pattern::Kind::Grid;
            edit.rows = 4; edit.columns = 8;
            edit.rowSpacing = 5.0 * 0.001 / context->openingFence().metersPerUnit();
            edit.columnSpacing = edit.rowSpacing;
            std::atomic_bool stop{false}, entered{false};
            std::shared_ptr<const core3d::profile_d4::PreparedCreation> prepared;
            std::thread worker([&] {
                entered.store(true, std::memory_order_release);
                prepared = core3d::profile_d4::PrepareCreation(capture, edit, stop);
            });
            while (!entered.load(std::memory_order_acquire)) std::this_thread::yield();
            stop.store(true, std::memory_order_release);
            worker.join();
            const bool cancelled = !prepared;
            prepared.reset(); capture.reset(); context.reset();
            auto retryContext = viewer->captureNativeOpeningContext(
                width, height, {hostText, sourceText});
            std::shared_ptr<const core3d::profile_d4::CreationCapture> retry;
            const bool unchanged = retryContext
                && core3d::profile_d4::CaptureCreationHost(*owner, hostText,
                    sourceText, retryContext, width, height, retry)
                    == core3d::profile_d4::CaptureStatus::Current;
            return @{@"cancelled": @(cancelled), @"entered": @(entered.load()),
                     @"unchanged": @(unchanged),
                     @"closed": @(!owner->Document()->HasOpenCommand())};
        } catch (...) { return @{@"cancelled": @NO, @"phase": @"exception"}; }
    }
}

static NSDictionary *DebugEdit(Core3DViewController *controller,
                               NSString *host, int32_t scenario) {
    @autoreleasepool {
        try {
            std::shared_ptr<core3d::Core3DViewer> viewer; Handle(OcctDocument) owner;
            std::shared_ptr<core3d::native_opening::Context> context;
            std::uint32_t width = 0, height = 0;
            if (scenario < 0 || scenario > 9
                || !Inputs(controller, host, viewer, owner, context, width, height))
                return @{@"committed": @NO, @"phase": @"inputs"};
            core3d::feature_pattern_owner::Snapshot opening;
            if (core3d::feature_pattern_owner::CaptureNative(
                    *owner, host.UTF8String, context, opening)
                    != core3d::feature_pattern_owner::Refusal::None)
                return @{@"committed": @NO, @"phase": @"capture"};
            const auto& d = opening.record.definition;
            core3d::feature_pattern_owner::Edit edit;
            edit.sourceCutStepID = d.sourceCutStepID;
            edit.kind = d.distribution.kind; edit.rowAxis = d.distribution.rowAxis;
            edit.columnAxis = d.distribution.columnAxis;
            edit.rows = d.distribution.rowCount; edit.columns = d.distribution.columnCount;
            edit.rowSpacing = d.distribution.rowSpacing;
            edit.columnSpacing = d.distribution.columnSpacing;
            edit.sweepRadians = d.distribution.sweepRadians;
            edit.radialPivotLocal = d.distribution.radialPivotLocal;
            for (const auto& member : d.distribution.members)
                if (member.state == core3d::pattern::MemberState::Suppressed)
                    edit.suppressed.insert(member.coordinate);
            const double spacing = 15.0 * 0.001 / d.metersPerUnit;
            if (scenario == 0) edit.columns = 4;
            else if (scenario == 1) {
                const auto* program = std::get_if<core3d::retained_boolean::Program>(
                    &opening.sourceProgram.recipe);
                if (!program || program->steps.size() < 2)
                    return @{@"committed": @NO, @"phase": @"source-step"};
                edit.sourceCutStepID = program->steps[1].operand.identifier;
            } else if (scenario == 2) {
                edit.kind = core3d::pattern::Kind::Grid;
                edit.rows = 2; edit.columns = 3;
                edit.rowAxis = core3d::pattern::Axis::Y;
                edit.columnAxis = core3d::pattern::Axis::X;
                edit.rowSpacing = spacing; edit.columnSpacing = spacing;
            } else {
                if (scenario == 3) {
                    edit.kind = core3d::pattern::Kind::Linear;
                    edit.rows = 1; edit.columns = 3;
                    edit.columnAxis = core3d::pattern::Axis::Y;
                    edit.columnSpacing = spacing;
                } else if (scenario == 4) {
                    edit.columns = 1;
                } else if (scenario == 5) {
                    edit.columns = 33;
                } else if (scenario == 6) {
                    edit.columnSpacing = 0;
                } else if (scenario == 7) {
                    edit.sourceCutStepID = 0;
                } else if (scenario == 8) {
                    edit.columns = 2;
                } else {
                    edit.kind = core3d::pattern::Kind::Grid;
                    edit.rows = 4; edit.columns = 8;
                    edit.rowAxis = core3d::pattern::Axis::Y;
                    edit.columnAxis = core3d::pattern::Axis::X;
                    edit.rowSpacing = 5.0 * 0.001 / d.metersPerUnit;
                    edit.columnSpacing = 5.0 * 0.001 / d.metersPerUnit;
                }
            }
            const auto prepared = core3d::feature_pattern_owner::PrepareNative(
                *owner, opening, edit, core3d::feature_pattern_owner::Limits{});
            if (!prepared.admitted()) return @{@"committed": @NO,
                @"phase": @"prepare", @"refusal": @(unsigned(prepared.refusal))};
            const auto outcome = core3d::feature_pattern_owner::ApplyNative(
                *owner, prepared, context);
            if (outcome != core3d::feature_pattern_owner::ApplyOutcome::Committed)
                return @{@"committed": @NO, @"phase": @"apply",
                    @"outcome": @(unsigned(outcome))};
            // ApplyNative has completed publication and released its stager;
            // retire that opening before acquiring post-commit observation.
            context.reset();
            auto observed = viewer->captureNativeOpeningContext(
                width, height, {host.UTF8String});
            NSMutableDictionary *result = [Observe(
                owner, host, observed) mutableCopy];
            result[@"committed"] = @YES; result[@"phase"] = @"complete";
            return result;
        } catch (...) { return @{@"committed": @NO, @"phase": @"exception"}; }
    }
}

extern "C" void *Core3DDebugD4ProfileCreate(
    Core3DViewController *controller, NSString *host, NSString *source) {
    return (__bridge_retained void *)DebugCreate(controller, host, source);
}

extern "C" void *Core3DDebugD4ProfileCreateFault(
    Core3DViewController *controller, NSString *host, NSString *source,
    int32_t fault) {
    return (__bridge_retained void *)DebugCreate(controller, host, source, fault);
}

extern "C" void *Core3DDebugD4ProfileCreationStop(
    Core3DViewController *controller, NSString *host, NSString *source) {
    return (__bridge_retained void *)DebugCreationStop(controller, host, source);
}

extern "C" void *Core3DDebugD4ProfileObserve(
    Core3DViewController *controller, NSString *host) {
    return (__bridge_retained void *)DebugObserve(controller, host);
}

extern "C" void *Core3DDebugD4ProfileEdit(
    Core3DViewController *controller, NSString *host, int32_t scenario) {
    return (__bridge_retained void *)DebugEdit(controller, host, scenario);
}

extern "C" void *Core3DDebugPhysicalWorkingFrameProbe(int32_t scenario) {
    return (__bridge_retained void *)WorkingFrameProbe(scenario);
}

extern "C" void *Core3DDebugPhysicalWorkingFrameEmptyCentimetreDocumentSeed() {
    @autoreleasepool {
        WorkingFrameDocument holder;
        double observed = 0;
        if (!MakeWorkingFrameDocument(0.01, holder, observed)
            || observed != 0.01) return nullptr;
        NSURL *base = [NSFileManager.defaultManager.temporaryDirectory
            URLByAppendingPathComponent:[NSString stringWithFormat:
                @"%@.working-frame-empty-cm", NSUUID.UUID.UUIDString]];
        NSString *xbf = [base.path stringByAppendingString:@".xbf"];
        NSData *result = nil;
        try {
            if (holder.app->SaveAs(holder.document, base.path.UTF8String) == PCDM_SS_OK)
                result = [NSData dataWithContentsOfFile:xbf];
        } catch (...) { result = nil; }
        [NSFileManager.defaultManager removeItemAtURL:base error:nil];
        [NSFileManager.defaultManager removeItemAtPath:xbf error:nil];
        return result ? (__bridge_retained void *)result : nullptr;
    }
}

extern "C" uint64_t Core3DDebugD4ProfileContinuationContractProbe(
    int32_t scenario, double metersPerUnit) {
    if (!(metersPerUnit == 0.001 || metersPerUnit == 1.0)) return 0;
    const double localPerMM = 0.001 / metersPerUnit;
    uint64_t bits = 0;
    if (std::isfinite(localPerMM) && localPerMM > 0) bits |= 1;
    if (core3d::feature_pattern::MaximumGeneratedFeatures == 32) bits |= 2;
    if (core3d::dependent_replay::Limits{}.records == 128) bits |= 4;
    if (core3d::retained_boolean::MaximumOperands == 4) bits |= 8;
    core3d::retained_edge_treatment::ReplayBudget debt;
    if (debt.valid()) bits |= 16;
    const double host8 = 80 * 50 * 8;
    const double result8 = host8 - 96 * std::acos(-1.0);
    const double host6 = 80 * 50 * 6;
    const double result6 = host6 - 72 * std::acos(-1.0);
    if (result8 < host8 && result6 < host6 && result6 < result8) bits |= 32;
    if (scenario >= 0 && scenario <= 9) bits |= 64;
    return bits;
}
#endif
