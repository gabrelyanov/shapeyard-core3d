#import "Core3DAuthoredParameterOpening.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/AuthoredParameterNativeOwner.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "Core3DViewer.h"
#if DEBUG
#include "../OCCTKit/PartBooleanAnalyticPlacementProof.hxx"
#include "../OCCTKit/PartBooleanCorrespondence.hxx"
#include "../OCCTKit/CompositeRecipeCodec.hxx"
#include <cstring>
#endif

#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <cmath>
#include <memory>

@interface Core3DRectangularLoftDefinition ()
- (core3d::rectangular_loft::Definition)nativeDefinition;
@end

namespace {
using NativeOwner = core3d::authored_parameter::Owner;

NSString *AuthoredIdentifier(const core3d::retained_recipe::UUID& value) {
    return [[NSUUID alloc] initWithUUIDBytes:value.data()].UUIDString;
}

bool SameExpectedScene(Core3DSceneSnapshot *expected, Core3DSceneSnapshot *current,
                       NSString *entity) noexcept {
    if (!expected || !current || !entity
        || ![expected.publicationSourceIdentifier isEqualToString:
             current.publicationSourceIdentifier]
        || expected.revisions.documentGeneration != current.revisions.documentGeneration
        || expected.revisions.modelRevision != current.revisions.modelRevision
        || expected.revisions.presentationRevision != current.revisions.presentationRevision
        || current.selectionMode != Core3DSceneElementKindObject
        || current.selection.selectedElements.count != 1) return false;
    Core3DSceneElementIdentifier *selected = current.selection.selectedElements.firstObject;
    Core3DSceneElementIdentifier *observed = expected.selection.selectedElements.firstObject;
    return expected.selection.selectedElements.count == 1
        && selected.kind == Core3DSceneElementKindObject
        && observed.kind == selected.kind
        && [selected.entityIdentifier isEqualToString:entity]
        && [observed.entityIdentifier isEqualToString:entity]
        && observed.topologyIndex == selected.topologyIndex
        && observed.geometryRevision == selected.geometryRevision;
}

TDF_Label ExactRoot(const Handle(OcctDocument)& document,
                    const std::string& entity) noexcept {
    try {
        if (document.IsNull() || document->Document().IsNull() || entity.empty()) return {};
        const auto shapes = XCAFDoc_DocumentTool::ShapeTool(document->Document()->Main());
        if (shapes.IsNull()) return {};
        TDF_LabelSequence roots; shapes->GetFreeShapes(roots);
        TDF_Label result; unsigned matches = 0;
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            if (document->EntityIdentifierForLabel(roots.Value(index)) == entity) {
                result = roots.Value(index); ++matches;
            }
        }
        return matches == 1 ? result : TDF_Label();
    } catch (...) { return {}; }
}

Core3DAuthoredParameterPreparationResult PublicResult(
    core3d::authored_parameter::Refusal refusal) noexcept {
    using Refusal = core3d::authored_parameter::Refusal;
    switch (refusal) {
        case Refusal::MalformedMutation:
        case Refusal::WrongThread:
            return Core3DAuthoredParameterPreparationResultMalformedMutation;
        case Refusal::ForeignOpening:
            return Core3DAuthoredParameterPreparationResultForeignOpening;
        case Refusal::StaleOpening:
            return Core3DAuthoredParameterPreparationResultStaleOpening;
        case Refusal::Cancelled:
            return Core3DAuthoredParameterPreparationResultCancelled;
        case Refusal::RecoveryRequired:
            return Core3DAuthoredParameterPreparationResultRecoveryRequired;
        case Refusal::None:
        case Refusal::UnsupportedCapability:
            return Core3DAuthoredParameterPreparationResultUnsupportedCapability;
    }
}

core3d::authored_loft::Field NativeLoftField(Core3DAuthoredLoftField field) noexcept {
    using Field = core3d::authored_loft::Field;
    switch (field) {
        case Core3DAuthoredLoftFieldStationZ: return Field::StationZ;
        case Core3DAuthoredLoftFieldStationCenterX: return Field::StationCenterX;
        case Core3DAuthoredLoftFieldStationCenterY: return Field::StationCenterY;
        case Core3DAuthoredLoftFieldFrameTranslationX: return Field::FrameTranslationX;
        case Core3DAuthoredLoftFieldFrameTranslationY: return Field::FrameTranslationY;
        case Core3DAuthoredLoftFieldFrameTranslationZ: return Field::FrameTranslationZ;
        case Core3DAuthoredLoftFieldFrameQuaternionX: return Field::FrameQuaternionX;
        case Core3DAuthoredLoftFieldFrameQuaternionY: return Field::FrameQuaternionY;
        case Core3DAuthoredLoftFieldFrameQuaternionZ: return Field::FrameQuaternionZ;
        case Core3DAuthoredLoftFieldFrameQuaternionW: return Field::FrameQuaternionW;
        case Core3DAuthoredLoftFieldFrameSignedScale: return Field::FrameSignedScale;
    }
}

NSString *LoftStatusCode(core3d::authored_loft::Status status) {
    using Status = core3d::authored_loft::Status;
    switch (status) {
        case Status::Prepared: return @"authored.none";
        case Status::Unchanged: return @"authored.unchanged";
        case Status::Malformed: return @"authored.loft.malformed";
        case Status::UnsupportedSource: return @"authored.loft.unsupported-source";
        case Status::IdentityChanged: return @"authored.loft.identity-changed";
        case Status::FieldChanged: return @"authored.loft.field-changed";
        case Status::InvalidDefinition: return @"authored.loft.invalid-definition";
    }
}

Core3DPartBooleanValues *BooleanPublicValues(
    const core3d::part_boolean::AnalyticDefinition& definition) {
    NSMutableArray<Core3DPartBooleanInputValues *> *inputs =
        [NSMutableArray arrayWithCapacity:2];
    for (std::size_t index = 0; index < 2; ++index) {
        const auto& value = definition.inputs[index];
        const double mm = value.metersPerUnit * 1000.0;
        if (!std::isfinite(mm) || mm <= 0) return nil;
        NSString *name = [[NSString alloc] initWithBytes:value.originalName.data()
            length:value.originalName.size() encoding:NSUTF8StringEncoding];
        Core3DPartBooleanInputValues *input = [[Core3DPartBooleanInputValues alloc]
            initWithRole:index == 0 ? @"left" : @"right" name:name ?: @""
            dimensionsMM:simd_make_double3(value.dimensions[0] * mm,
                value.dimensions[1] * mm, value.dimensions[2] * mm)
            translationMM:simd_make_double3(value.translation[0] * mm,
                value.translation[1] * mm, value.translation[2] * mm)
            rotationXYZW:simd_make_double4(value.rotationXYZW[0], value.rotationXYZW[1],
                value.rotationXYZW[2], value.rotationXYZW[3])
            metersPerUnit:value.metersPerUnit
            baseColorSRGB:simd_make_double3(value.originalMaterial.baseColorSRGB[0],
                value.originalMaterial.baseColorSRGB[1], value.originalMaterial.baseColorSRGB[2])
            metallic:value.originalMaterial.metallic roughness:value.originalMaterial.roughness];
        if (!input) return nil;
        [inputs addObject:input];
    }
    return [[Core3DPartBooleanValues alloc] initWithOperation:
        static_cast<Core3DPartBooleanOperation>(definition.operation) inputs:inputs];
}

double NativeLength(double publicValue, double original, double mm) noexcept {
    // A value returned by BooleanPublicValues is an immutable native original.
    // Preserve it instead of dividing the presentation value back into native
    // units, including the sign bit of zero.
    return publicValue == original * mm ? original : publicValue / mm;
}

bool BooleanRequest(const core3d::part_boolean::AnalyticDefinition& original,
                    Core3DPartBooleanValues *values,
                    core3d::part_boolean::AnalyticDefinition& requested) noexcept {
    requested = original;
    try {
        if (![values isKindOfClass:Core3DPartBooleanValues.class]
            || values.inputs.count != 2
            || values.operation < Core3DPartBooleanOperationUnion
            || values.operation > Core3DPartBooleanOperationIntersect) return false;
        requested.operation = static_cast<core3d::part_boolean::Operation>(values.operation);
        for (NSUInteger index = 0; index < 2; ++index) {
            Core3DPartBooleanInputValues *input = values.inputs[index];
            const auto& before = original.inputs[index];
            NSString *role = index == 0 ? @"left" : @"right";
            NSString *name = [[NSString alloc] initWithBytes:before.originalName.data()
                length:before.originalName.size() encoding:NSUTF8StringEncoding];
            if (![input isKindOfClass:Core3DPartBooleanInputValues.class]
                || input.family != Core3DPartBooleanInputFamilyAnalyticRectangularPrism
                || ![input.role isEqualToString:role] || ![input.name isEqualToString:name]
                || input.metersPerUnit != before.metersPerUnit
                || input.baseColorSRGB.x != before.originalMaterial.baseColorSRGB[0]
                || input.baseColorSRGB.y != before.originalMaterial.baseColorSRGB[1]
                || input.baseColorSRGB.z != before.originalMaterial.baseColorSRGB[2]
                || input.metallic != before.originalMaterial.metallic
                || input.roughness != before.originalMaterial.roughness) return false;
            const double mm = before.metersPerUnit * 1000.0;
            if (!std::isfinite(mm) || mm <= 0) return false;
            auto& after = requested.inputs[index];
            after.dimensions = {NativeLength(input.dimensionsMM.x, before.dimensions[0], mm),
                                NativeLength(input.dimensionsMM.y, before.dimensions[1], mm),
                                NativeLength(input.dimensionsMM.z, before.dimensions[2], mm)};
            after.translation = {NativeLength(input.translationMM.x, before.translation[0], mm),
                                 NativeLength(input.translationMM.y, before.translation[1], mm),
                                 NativeLength(input.translationMM.z, before.translation[2], mm)};
            after.rotationXYZW = {input.rotationXYZW.x, input.rotationXYZW.y,
                                  input.rotationXYZW.z, input.rotationXYZW.w};
        }
        return core3d::part_boolean::Valid(requested);
    } catch (...) { requested = {}; return false; }
}

#if DEBUG
using RotationQuaternion = std::array<double, 4>;

RotationQuaternion RotationOracleQuaternion(std::size_t kind) {
    const double angle = kind == 4 ? std::acos(-1.0) / 7.0
                                   : std::acos(-1.0) / (8.0 + kind);
    const double sine = std::sin(angle / 2), cosine = std::cos(angle / 2);
    if (kind == 1) return {{sine, 0, 0, cosine}};
    if (kind == 2) return {{0, sine, 0, cosine}};
    if (kind == 3) return {{0, 0, sine, cosine}};
    const double inverseRootSix = 1.0 / std::sqrt(6.0);
    return {{sine * inverseRootSix, sine * 2 * inverseRootSix,
             sine * inverseRootSix, cosine}};
}

core3d::part_boolean::AnalyticDefinition RotationOracleFixture(
    double unit, core3d::part_boolean::Operation operation,
    std::size_t rotatedInput, const RotationQuaternion& rotation) {
    namespace pb = core3d::part_boolean;
    pb::AnalyticDefinition value;
    value.operation = operation;
    const auto reads = pb::build::FixtureReads();
    const std::array<core3d::retained_recipe::DependencyRead, 2> inputs{
        reads.leftSource, reads.rightSource};
    const std::array<std::array<double, 3>, 2> dimensions{{
        {{12, 9, 7}}, {{9, 7, 5}}}};
    const std::array<core3d::part_boolean::analytic_placement_proof::Vec, 2> centres{{
        {0, 0, 0}, {1.3, 0.7, 0.4}}};
    for (std::size_t index = 0; index < 2; ++index) {
        auto& input = value.inputs[index];
        const auto& read = inputs[index];
        input.rootNode = read.locator.node;
        input.originalSourceFeature = read.locator.sourceFeature;
        input.commitments = {read.geometry, read.recipe, read.placement,
                             read.material, read.groups};
        input.dimensions = dimensions[index];
        input.rotationXYZW = index == rotatedInput
            ? rotation : RotationQuaternion{{0, 0, 0, 1}};
        const auto half = pb::analytic_placement_proof::Rotate(
            input.rotationXYZW, {input.dimensions[0] / 2,
                                 input.dimensions[1] / 2,
                                 input.dimensions[2] / 2});
        input.translation = {{centres[index].x - half.x,
                              centres[index].y - half.y,
                              centres[index].z - half.z}};
        input.metersPerUnit = unit;
        input.originalMaterial.identifier.fill(std::uint8_t(90 + index));
        input.originalMaterial.baseColorSRGB = index == 0
            ? std::array<double, 4>{{0.8, 0.2, 0.1, 1}}
            : std::array<double, 4>{{0.1, 0.3, 0.9, 1}};
        input.originalName = index == 0 ? "left-rotation-probe" : "right-rotation-probe";
    }
    return value;
}

bool SameQuaternionBits(const RotationQuaternion& value,
    const std::array<std::uint64_t, 4>& bits) noexcept {
    for (std::size_t index = 0; index < 4; ++index) {
        std::uint64_t expected = 0;
        std::memcpy(&expected, &value[index], sizeof(double));
        if (expected != bits[index]) return false;
    }
    return true;
}

NSDictionary *RotationOracleProbe(int32_t scenario) {
    @autoreleasepool {
        namespace pb = core3d::part_boolean;
        namespace proof = pb::analytic_placement_proof;
        NSMutableDictionary *answer = [NSMutableDictionary dictionary];
        const auto reads = pb::build::FixtureReads();
        if (scenario == 0) {
            bool all = true;
            std::size_t count = 0, maximumWork = 0, minimumCells = SIZE_MAX;
            NSDictionary *firstFailure = nil;
            for (double unit : {0.001, 1.0}) {
                for (std::size_t rotatedInput = 0; rotatedInput < 2; ++rotatedInput) {
                    for (pb::Operation operation : {pb::Operation::Union,
                         pb::Operation::Subtract, pb::Operation::Intersect}) {
                        for (std::size_t rotation = 1; rotation <= 4; ++rotation) {
                            const auto definition = RotationOracleFixture(unit, operation,
                                rotatedInput, RotationOracleQuaternion(rotation));
                            const auto built = pb::build::BuildAnalytic(
                                definition, unit, reads, reads);
                            const auto proven = proof::Prove(definition, built, unit);
                            if ((!built.complete || !proven.proven()) && !firstFailure) {
                                firstFailure = @{
                                    @"unit": @(unit), @"role": @(rotatedInput),
                                    @"operation": @(static_cast<int>(operation)),
                                    @"rotation": @(rotation), @"built": @(built.complete),
                                    @"source": @(proven.sourceEquations),
                                    @"volume": @(proven.operationOracle),
                                    @"planar": @(proven.completePlanarBoundary),
                                    @"correspondence": @(proven.correspondenceComplete),
                                    @"materials": @(proven.materialsComplete),
                                    @"solid": @(proven.connectedPositiveSolid),
                                    @"coverage": @(proven.completeBoundaryCoverage),
                                    @"orientation": @(proven.outwardOrientation),
                                    @"roles": @(proven.uniqueSourceAndCutRoles),
                                    @"bounded": @(proven.boundedArrangement),
                                    @"cells": @(proven.expectedCells),
                                    @"work": @(proven.work),
                                    @"expectedVolume": @(proven.expectedVolume),
                                    @"observedVolume": @(proven.observedVolume)};
                            }
                            all = all && built.complete && proven.proven();
                            maximumWork = std::max(maximumWork, proven.work);
                            minimumCells = std::min(minimumCells, proven.expectedCells);
                            ++count;
                        }
                    }
                }
            }
            answer[@"caseCount"] = @(count);
            answer[@"maximumWork"] = @(maximumWork);
            answer[@"minimumCells"] = @(minimumCells);
            answer[@"bothUnits"] = @(count == 48);
            answer[@"bothRolesAllOperationsXYZAndMixed"] = @(all);
            if (firstFailure) answer[@"firstFailure"] = firstFailure;
            answer[@"ok"] = @(all && count == 48
                && maximumWork <= proof::MaximumWork && minimumCells > 0);
            return answer;
        }
        if (scenario == 1) {
            bool positive = true, wrongBoundaryRejected = true;
            bool equalVolume = true, materialRejected = true;
            for (double unit : {0.001, 1.0}) {
                const auto definition = RotationOracleFixture(unit,
                    pb::Operation::Intersect, 1, RotationOracleQuaternion(4));
                const auto built = pb::build::BuildAnalytic(definition, unit, reads, reads);
                positive = positive && proof::Prove(definition, built, unit).proven();
                pb::build::AnalyticBuild displaced = built;
                gp_Trsf translation; translation.SetTranslation(gp_Vec(0.375, -0.25, 0.125));
                displaced.candidate.solid = BRepBuilderAPI_Transform(
                    built.candidate.solid, translation, Standard_True).Shape();
                const auto wrong = proof::Prove(definition, displaced, unit);
                wrongBoundaryRejected = wrongBoundaryRejected && !wrong.proven()
                    && !wrong.completeBoundaryCoverage;
                equalVolume = equalVolume && pb::correspondence::Near(
                    core3d::retained_part_boolean::Volume(displaced.candidate.solid),
                    core3d::retained_part_boolean::Volume(built.candidate.solid));
                auto wrongMaterial = definition;
                wrongMaterial.materialPolicy = static_cast<pb::MaterialPolicy>(255);
                materialRejected = materialRejected
                    && !proof::Prove(wrongMaterial, built, unit).proven();
            }
            answer[@"positive"] = @(positive);
            answer[@"wrongBoundaryRejected"] = @(wrongBoundaryRejected);
            answer[@"equalVolume"] = @(equalVolume);
            answer[@"materialRejected"] = @(materialRejected);
            answer[@"ok"] = @(positive && wrongBoundaryRejected
                && equalVolume && materialRejected);
            return answer;
        }
        bool exactBits = true, signDistinct = true, identityRejected = true;
        std::size_t count = 0;
        for (double unit : {0.001, 1.0}) {
            for (std::size_t rotatedInput = 0; rotatedInput < 2; ++rotatedInput) {
                for (std::size_t rotation = 1; rotation <= 4; ++rotation) {
                    const auto quaternion = RotationOracleQuaternion(rotation);
                    const auto definition = RotationOracleFixture(unit,
                        pb::Operation::Intersect, rotatedInput, quaternion);
                    const auto built = pb::build::BuildAnalytic(definition, unit, reads, reads);
                    const auto proven = proof::Prove(definition, built, unit);
                    exactBits = exactBits && proven.proven()
                        && SameQuaternionBits(quaternion,
                            proven.quaternionBits[rotatedInput]);
                    auto negative = quaternion;
                    for (double& coordinate : negative) coordinate = -coordinate;
                    const auto negativeDefinition = RotationOracleFixture(unit,
                        pb::Operation::Intersect, rotatedInput, negative);
                    const auto negativeBuilt = pb::build::BuildAnalytic(
                        negativeDefinition, unit, reads, reads);
                    const auto negativeProof = proof::Prove(
                        negativeDefinition, negativeBuilt, unit);
                    signDistinct = signDistinct && negativeProof.proven()
                        && SameQuaternionBits(negative,
                            negativeProof.quaternionBits[rotatedInput])
                        && negativeProof.quaternionBits[rotatedInput]
                            != proven.quaternionBits[rotatedInput];
                    ++count;
                }
            }
            const auto identity = RotationOracleFixture(unit,
                pb::Operation::Intersect, 0, {{0, 0, 0, -1}});
            const auto identityBuilt = pb::build::BuildAnalytic(identity, unit, reads, reads);
            identityRejected = identityRejected
                && !proof::Prove(identity, identityBuilt, unit).proven();
        }
        answer[@"caseCount"] = @(count);
        answer[@"exactBits"] = @(exactBits);
        answer[@"qAndNegativeQRemainBitDistinct"] = @(signDistinct);
        answer[@"negativeIdentityRejected"] = @(identityRejected);
        answer[@"ok"] = @(count == 16 && exactBits && signDistinct && identityRejected);
        return answer;
    }
}

NSDictionary *RotationCountedProbe(int32_t scenario) {
    @autoreleasepool {
        namespace pb = core3d::part_boolean;
        namespace proof = pb::analytic_placement_proof;
        namespace tb = core3d::retained_topology_budget;
        const auto reads = pb::build::FixtureReads();
        NSMutableDictionary *answer = [NSMutableDictionary dictionary];
        NSMutableArray *rows = [NSMutableArray array];
        bool all = true;
        for (double unit : {0.001, 1.0}) {
            const auto definition = RotationOracleFixture(unit,
                pb::Operation::Union, 0, RotationOracleQuaternion(4));
            if (scenario == 0) {
                tb::Counter budget;
                const std::atomic_bool cancelled(false);
                const auto replay = core3d::a1_analytic_input_adapter::
                    ReplayAuthoredCounted(definition, unit, reads, reads,
                                          budget, cancelled);
                const bool rowOK = replay.proven() && replay.usedRotationProof
                    && replay.rotationProof.proven()
                    && replay.fixedPoint.fixedPoint()
                    && budget.buildStages <= tb::MaximumBuildStages
                    && budget.topologyVisits <= tb::MaximumTopologyVisits;
                all = all && rowOK;
                [rows addObject:@{
                    @"unit": @(unit), @"proven": @(replay.proven()),
                    @"rotationProof": @(replay.rotationProof.proven()),
                    @"fixedPoint": @(replay.fixedPoint.fixedPoint()),
                    @"visits": @(budget.topologyVisits),
                    @"stages": @(budget.buildStages),
                    @"visitsC25": @(budget.visitsBySite[
                        std::size_t(tb::Site::C25AnalyticInput)]),
                    @"visitsC26": @(budget.visitsBySite[
                        std::size_t(tb::Site::C26AnalyticBoolean)]),
                    @"stagesC25": @(budget.stagesBySite[
                        std::size_t(tb::Site::C25AnalyticInput)]),
                    @"stagesC26": @(budget.stagesBySite[
                        std::size_t(tb::Site::C26AnalyticBoolean)])}];
                continue;
            }

            // Exercise the counted facade in isolation so the debt is known to
            // target its first expected-arrangement reservation, rather than a
            // producer pass earlier in the full replay.
            const auto built = pb::build::BuildAnalytic(
                definition, unit, reads, reads);
            tb::Counter deniedBudget;
            tb::debug::ArmDebtHook(tb::Site::C25AnalyticInput, 1,
                proof::MaximumWork - 1, tb::MaximumBuildStages);
            proof::Proof deniedProof;
            const std::atomic_bool running(false);
            const auto denied = proof::ProveCounted(definition, built, unit,
                deniedBudget, running, deniedProof);
            tb::debug::ClearDebtHook();
            const auto eventsAtStop = deniedBudget.chargeEvents;
            const auto visitsAtStop = deniedBudget.topologyVisits;
            const auto stagesAtStop = deniedBudget.buildStages;
            const bool laterCharge = deniedBudget.visit(1,
                tb::Site::C26AnalyticBoolean);
            tb::Counter cancelledBudget;
            proof::Proof cancelledProof;
            const std::atomic_bool cancelled(true);
            const auto cancelledStatus = proof::ProveCounted(definition, built,
                unit, cancelledBudget, cancelled, cancelledProof);
            const bool rowOK = built.complete
                && denied == tb::WalkStatus::BudgetDenied
                && deniedBudget.exhausted && !deniedProof.proven()
                && deniedBudget.firstDeniedSite == tb::Site::C25AnalyticInput
                && deniedBudget.firstDeniedRequested == proof::MaximumWork
                && !laterCharge && deniedBudget.chargeEvents == eventsAtStop
                && deniedBudget.topologyVisits == visitsAtStop
                && deniedBudget.buildStages == stagesAtStop
                && cancelledStatus == tb::WalkStatus::Cancelled
                && !cancelledProof.proven() && !cancelledBudget.exhausted
                && cancelledBudget.topologyVisits == 0
                && cancelledBudget.buildStages == 0;
            all = all && rowOK;
            [rows addObject:@{
                @"unit": @(unit), @"budgetDenied": @(denied
                    == tb::WalkStatus::BudgetDenied),
                @"sticky": @(deniedBudget.exhausted && !laterCharge),
                @"zeroAfterDenial": @(deniedBudget.chargeEvents == eventsAtStop
                    && deniedBudget.topologyVisits == visitsAtStop
                    && deniedBudget.buildStages == stagesAtStop),
                @"deniedSite": @(static_cast<int>(deniedBudget.firstDeniedSite)),
                @"deniedRequested": @(deniedBudget.firstDeniedRequested),
                @"cancelled": @(cancelledStatus == tb::WalkStatus::Cancelled),
                @"cancelledZeroWork": @(cancelledBudget.topologyVisits == 0
                    && cancelledBudget.buildStages == 0)}];
        }
        answer[@"rows"] = rows;
        answer[@"bothUnits"] = @(rows.count == 2);
        answer[@"ok"] = @(all && rows.count == 2);
        return answer;
    }
}
#endif
} // namespace

@interface Core3DAuthoredParameterApplyResult ()
- (instancetype)initWithOutcome:(Core3DAuthoredParameterApplyOutcome)outcome
    refusalCode:(NSString *)refusalCode measuredUndoDelta:(NSNumber *)measuredUndoDelta;
@end

@implementation Core3DAuthoredParameterApplyResult
- (instancetype)initWithOutcome:(Core3DAuthoredParameterApplyOutcome)outcome
    refusalCode:(NSString *)refusalCode measuredUndoDelta:(NSNumber *)measuredUndoDelta {
    self = [super init];
    if (self) {
        _outcome = outcome;
        _refusalCode = [refusalCode copy];
        _measuredUndoDelta = [measuredUndoDelta copy];
    }
    return self;
}
@end

@interface Core3DAuthoredParameterRecoveryResult ()
- (instancetype)initWithOutcome:(Core3DAuthoredParameterRecoveryOutcome)outcome
    refusalCode:(NSString *)refusalCode measuredUndoDelta:(NSNumber *)measuredUndoDelta
    recoveryPhase:(Core3DAuthoredParameterRecoveryPhase)recoveryPhase;
#if DEBUG
- (void)recordDebugCountersReconcileCalls:(NSUInteger)reconcileCalls
    observationCalls:(NSUInteger)observationCalls
    observationFaultHits:(NSUInteger)observationFaultHits
    publicationCalls:(NSUInteger)publicationCalls
    recoveryCommandCalls:(NSUInteger)recoveryCommandCalls;
#endif
@end

@implementation Core3DAuthoredParameterRecoveryResult
- (instancetype)initWithOutcome:(Core3DAuthoredParameterRecoveryOutcome)outcome
    refusalCode:(NSString *)refusalCode measuredUndoDelta:(NSNumber *)measuredUndoDelta
    recoveryPhase:(Core3DAuthoredParameterRecoveryPhase)recoveryPhase {
    self = [super init];
    if (self) {
        _outcome = outcome;
        _refusalCode = [refusalCode copy];
        _measuredUndoDelta = [measuredUndoDelta copy];
        _recoveryPhase = recoveryPhase;
#if DEBUG
        _reconcileCalls = 0;
        _observationCalls = 0;
        _observationFaultHits = 0;
        _publicationCalls = 0;
        _recoveryCommandCalls = 0;
#endif
    }
    return self;
}
#if DEBUG
- (void)recordDebugCountersReconcileCalls:(NSUInteger)reconcileCalls
    observationCalls:(NSUInteger)observationCalls
    observationFaultHits:(NSUInteger)observationFaultHits
    publicationCalls:(NSUInteger)publicationCalls
    recoveryCommandCalls:(NSUInteger)recoveryCommandCalls {
    _reconcileCalls = reconcileCalls;
    _observationCalls = observationCalls;
    _observationFaultHits = observationFaultHits;
    _publicationCalls = publicationCalls;
    _recoveryCommandCalls = recoveryCommandCalls;
}
#endif
@end

//! A2-N3 host-retained native recovery issuer. The host keeps the exact
//! authored-parameter owner that produced an outcome-unknown Boolean apply so
//! the production continuation can settle the document-owned recovery record
//! after every client-held opening/operation wrapper has been released. The
//! owner holds no reference back to the host or the opening wrapper.
@interface Core3DAuthoredParameterRecoveryIssuer : NSObject {
@public
    std::shared_ptr<NativeOwner> _owner;
    const void *_ocafDocument;
    const void *_ocafData;
}
- (instancetype)initWithOwner:(std::shared_ptr<NativeOwner>)owner
    ocafDocument:(const void *)ocafDocument ocafData:(const void *)ocafData;
@end

@implementation Core3DAuthoredParameterRecoveryIssuer
- (instancetype)initWithOwner:(std::shared_ptr<NativeOwner>)owner
    ocafDocument:(const void *)ocafDocument ocafData:(const void *)ocafData {
    self = [super init];
    if (self) {
        _owner = std::move(owner);
        _ocafDocument = ocafDocument;
        _ocafData = ocafData;
    }
    return self;
}
@end

//! Per-host issuer registry: weak host keys, strong issuer values, touched
//! only on the main thread by the opening/apply/recovery paths below.
static NSMapTable<Core3DViewController *, Core3DAuthoredParameterRecoveryIssuer *>
    *Core3DAuthoredParameterRecoveryIssuers() {
    static NSMapTable *issuers;
    static dispatch_once_t once;
    dispatch_once(&once, ^{
        issuers = [NSMapTable weakToStrongObjectsMapTable];
    });
    return issuers;
}

#if DEBUG
@interface Core3DDebugAuthoredBooleanFaultReceipt ()
- (instancetype)initWithSite:(Core3DDebugAuthoredBooleanFaultSite)site
    delivery:(Core3DDebugAuthoredBooleanFaultDelivery)delivery
    hitCount:(NSUInteger)hitCount;
@end

@implementation Core3DDebugAuthoredBooleanFaultReceipt
- (instancetype)initWithSite:(Core3DDebugAuthoredBooleanFaultSite)site
    delivery:(Core3DDebugAuthoredBooleanFaultDelivery)delivery
    hitCount:(NSUInteger)hitCount {
    self = [super init];
    if (self) { _site = site; _delivery = delivery; _hitCount = hitCount; }
    return self;
}
@end

@interface Core3DDebugTreatmentNamingVersionRow ()
- (instancetype)initWithLabelEntry:(NSString *)labelEntry version:(NSInteger)version;
@end

@implementation Core3DDebugTreatmentNamingVersionRow
- (instancetype)initWithLabelEntry:(NSString *)labelEntry version:(NSInteger)version {
    self = [super init];
    if (self) { _labelEntry = [labelEntry copy]; _version = version; }
    return self;
}
@end

@interface Core3DDebugAuthoredBooleanObservation ()
- (instancetype)initWithNamingVersions:
    (NSArray<Core3DDebugTreatmentNamingVersionRow *> *)namingVersions
    availableUndos:(NSInteger)availableUndos availableRedos:(NSInteger)availableRedos
    openCommand:(BOOL)openCommand pendingCompanion:(BOOL)pendingCompanion
    pendingAfterCaptured:(BOOL)pendingAfterCaptured
    finalizedCompanions:(NSUInteger)finalizedCompanions;
@end

@implementation Core3DDebugAuthoredBooleanObservation
- (instancetype)initWithNamingVersions:
    (NSArray<Core3DDebugTreatmentNamingVersionRow *> *)namingVersions
    availableUndos:(NSInteger)availableUndos availableRedos:(NSInteger)availableRedos
    openCommand:(BOOL)openCommand pendingCompanion:(BOOL)pendingCompanion
    pendingAfterCaptured:(BOOL)pendingAfterCaptured
    finalizedCompanions:(NSUInteger)finalizedCompanions {
    self = [super init];
    if (self) {
        _namingVersions = [namingVersions copy];
        _availableUndos = availableUndos;
        _availableRedos = availableRedos;
        _hasOpenCommand = openCommand;
        _hasPendingCompanion = pendingCompanion;
        _pendingCompanionAfterCaptured = pendingAfterCaptured;
        _finalizedCompanionCount = finalizedCompanions;
    }
    return self;
}
@end
#endif

@interface Core3DAuthoredParameterOperation () {
@public
    std::shared_ptr<core3d::retained_edge_treatment::r2::Work> _work;
    void (^_completion)(Core3DAuthoredParameterApplyResult *);
    BOOL _settled;
}
- (instancetype)initWithWork:
    (std::shared_ptr<core3d::retained_edge_treatment::r2::Work>)work
    completion:(void(^)(Core3DAuthoredParameterApplyResult *))completion;
@end

@implementation Core3DAuthoredParameterOperation
- (instancetype)initWithWork:
    (std::shared_ptr<core3d::retained_edge_treatment::r2::Work>)work
    completion:(void(^)(Core3DAuthoredParameterApplyResult *))completion {
    self = [super init];
    if (self) { _work = std::move(work); _completion = [completion copy]; }
    return self;
}
- (BOOL)cancel {
    if (![NSThread isMainThread] || _settled) return NO;
    _settled = YES;
    if (_work) core3d::Core3DViewer::cancelEdgeTreatmentR2(_work);
    auto completion = _completion; _completion = nil;
    if (completion) completion([[Core3DAuthoredParameterApplyResult alloc]
        initWithOutcome:Core3DAuthoredParameterApplyOutcomeCancelled
        refusalCode:@"authored.cancelled" measuredUndoDelta:@0]);
    return YES;
}
@end

@interface Core3DAuthoredParameterOpening () {
@public
    std::shared_ptr<NativeOwner> _nativeOwner;
    NSString *_documentIdentifier;
    NSString *_entityIdentifier;
    NSString *_definitionIdentifier;
    NSData *_sourceCanonicalBytes;
    NSData *_suffixCanonicalBytes;
    double _metersPerUnit;
}
- (instancetype)initWithNativeOwner:(std::shared_ptr<NativeOwner>)owner;
@end

@implementation Core3DAuthoredParameterOpening
- (instancetype)initWithNativeOwner:(std::shared_ptr<NativeOwner>)owner {
    self = [super init];
    if (!self || !owner) return nil;
    _nativeOwner = std::move(owner);
    const auto& capture = _nativeOwner->capture();
    _documentIdentifier = [AuthoredIdentifier(capture.owner().document) copy];
    _entityIdentifier = [AuthoredIdentifier(capture.owner().entity) copy];
    _definitionIdentifier = [AuthoredIdentifier(capture.owner().definition) copy];
    _sourceCanonicalBytes = [[NSData alloc] initWithBytes:capture.sourceBytes().data()
                                                   length:capture.sourceBytes().size()];
    _suffixCanonicalBytes = [[NSData alloc] initWithBytes:capture.suffixBytes().data()
                                                   length:capture.suffixBytes().size()];
    _metersPerUnit = capture.metersPerUnit();
    return self;
}
- (NSString *)documentIdentifier { return _documentIdentifier; }
- (NSString *)entityIdentifier { return _entityIdentifier; }
- (NSString *)definitionIdentifier { return _definitionIdentifier; }
- (NSData *)sourceCanonicalBytes { return _sourceCanonicalBytes; }
- (NSData *)suffixCanonicalBytes { return _suffixCanonicalBytes; }
- (double)metersPerUnit { return _metersPerUnit; }
- (Core3DAuthoredParameterOpeningCurrentness)currentness {
    using Currentness = core3d::authored_parameter::Currentness;
    if (!_nativeOwner) return Core3DAuthoredParameterOpeningCurrentnessCancelled;
    switch (_nativeOwner->currentness()) {
        case Currentness::Current: return Core3DAuthoredParameterOpeningCurrentnessCurrent;
        case Currentness::Stale: return Core3DAuthoredParameterOpeningCurrentnessStale;
        case Currentness::Cancelled: return Core3DAuthoredParameterOpeningCurrentnessCancelled;
        case Currentness::Recovery: return Core3DAuthoredParameterOpeningCurrentnessRecovery;
    }
}
@end

@implementation Core3DViewController (AuthoredParameterOpening)
- (Core3DAuthoredParameterRecoveryResult *)recoverAuthoredBooleanParameter {
    Core3DAuthoredParameterRecoveryOutcome outcome =
        Core3DAuthoredParameterRecoveryOutcomeNotOwner;
    NSString *code = @"authored.recovery-not-owner";
    NSNumber *delta = nil;
    Core3DAuthoredParameterRecoveryPhase phase =
        Core3DAuthoredParameterRecoveryPhaseNone;
#if DEBUG
    NSUInteger reconcileCalls = 0, observationCalls = 0, observationFaultHits = 0;
    NSUInteger publicationCalls = 0, recoveryCommandCalls = 0;
#endif
    if ([NSThread isMainThread]) {
        GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        const auto viewer = gl ? gl.viewer : nullptr;
        const Handle(OcctDocument) document = viewer
            ? viewer->getDocument() : Handle(OcctDocument)();
        Core3DAuthoredParameterRecoveryIssuer *issuer =
            [Core3DAuthoredParameterRecoveryIssuers() objectForKey:self];
        // A2-N3: only the retained issuer of the one outstanding transaction
        // may continue, and only while it still belongs to this host's exact
        // current native document, data and generation. A foreign or replaced
        // host/document, a missing issuer and a superseded authority all fail
        // closed as not-owner without observing or mutating the document.
        const Handle(TDocStd_Document) ocafDocument =
            document.IsNull() ? Handle(TDocStd_Document)() : document->Document();
        const void *ocafData = ocafDocument.IsNull()
            ? nullptr : ocafDocument->GetData().get();
        if (!document.IsNull() && !ocafDocument.IsNull() && issuer && issuer->_owner
            && issuer->_ocafDocument == ocafDocument.get()
            && issuer->_ocafData == ocafData
            && issuer->_owner->belongsTo(document)) {
            const auto result = issuer->_owner->reconcileBoolean();
            using RecoveryOutcome = core3d::authored_parameter::BooleanRecoveryOutcome;
            switch (result.outcome) {
                case RecoveryOutcome::Committed:
                    outcome = Core3DAuthoredParameterRecoveryOutcomeCommitted;
                    phase = Core3DAuthoredParameterRecoveryPhaseCompanionSettled;
                    break;
                case RecoveryOutcome::PriorRestored:
                    outcome = Core3DAuthoredParameterRecoveryOutcomePriorRestored;
                    phase = Core3DAuthoredParameterRecoveryPhaseCompanionSettled;
                    break;
                case RecoveryOutcome::RecoveryRequired:
                    // The retained record is still pending native
                    // classification; the lock stays and no delta is exposed.
                    outcome = Core3DAuthoredParameterRecoveryOutcomeRecoveryRequired;
                    phase = Core3DAuthoredParameterRecoveryPhasePending;
                    break;
                case RecoveryOutcome::NotOwner:
                    outcome = Core3DAuthoredParameterRecoveryOutcomeNotOwner;
                    phase = Core3DAuthoredParameterRecoveryPhaseNone;
                    break;
            }
            code = [NSString stringWithUTF8String:result.code.c_str()];
            if (result.measuredUndoDelta
                && (outcome == Core3DAuthoredParameterRecoveryOutcomeCommitted
                    || outcome == Core3DAuthoredParameterRecoveryOutcomePriorRestored))
                delta = @(*result.measuredUndoDelta);
#if DEBUG
            reconcileCalls = result.reconcileCalls;
            observationCalls = result.observationCalls;
            observationFaultHits = result.observationFaultHits;
            publicationCalls = result.publicationCalls;
            recoveryCommandCalls = result.recoveryCommandCalls;
#endif
        }
    }
    Core3DAuthoredParameterRecoveryResult *receipt =
        [[Core3DAuthoredParameterRecoveryResult alloc]
            initWithOutcome:outcome refusalCode:code measuredUndoDelta:delta
            recoveryPhase:phase];
#if DEBUG
    [receipt recordDebugCountersReconcileCalls:reconcileCalls
        observationCalls:observationCalls observationFaultHits:observationFaultHits
        publicationCalls:publicationCalls recoveryCommandCalls:recoveryCommandCalls];
#endif
    return receipt;
}

- (Core3DAuthoredParameterOpening *)openAuthoredParameterForEntityIdentifier:
    (NSString *)entityIdentifier expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread] || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128) return nil;
    try {
        GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl ? gl.viewer : nullptr;
        const CGSize size = self.viewportDrawableSize;
        Core3DSceneSnapshot *current = [self captureSceneSnapshot];
        if (!viewer || !SameExpectedScene(expected, current, entityIdentifier)
            || !std::isfinite(size.width) || !std::isfinite(size.height)
            || size.width < 1 || size.height < 1
            || size.width > UINT32_MAX || size.height > UINT32_MAX) return nil;
        const Handle(OcctDocument) document = viewer->getDocument();
        const std::string entity = entityIdentifier.UTF8String ?: "";
        const TDF_Label ownerLabel = ExactRoot(document, entity);
        if (ownerLabel.IsNull()) return nil;
        const std::uint32_t width = std::uint32_t(size.width);
        const std::uint32_t height = std::uint32_t(size.height);
        auto context = viewer->captureNativeOpeningContext(width, height, {entity});
        if (!context) return nil;
        auto capture = document->CaptureAuthoredParameterAuthority(ownerLabel, *context);
        auto owner = NativeOwner::Open(document, std::move(context), std::move(capture),
                                       width, height);
        if (!owner) return nil;
        // A2-N3: a genuinely fresh native opening is only possible once no
        // recovery lock is pending, so any host-retained recovery issuer is
        // superseded here; a late duplicate continuation must never settle or
        // cancel the newer operation that follows this opening.
        [Core3DAuthoredParameterRecoveryIssuers() removeObjectForKey:self];
        return [[Core3DAuthoredParameterOpening alloc]
            initWithNativeOwner:std::move(owner)];
    } catch (...) { return nil; }
}

- (Core3DAuthoredParameterPreparationResult)prepareAuthoredParameterOpening:
    (Core3DAuthoredParameterOpening *)opening
    mutationKind:(Core3DAuthoredParameterMutationKind)mutationKind {
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner)
        return Core3DAuthoredParameterPreparationResultMalformedMutation;
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    if (!opening->_nativeOwner->belongsTo(document))
        return Core3DAuthoredParameterPreparationResultForeignOpening;
    core3d::authored_parameter::Mutation mutation;
    mutation.capability = static_cast<core3d::authored_parameter::Capability>(mutationKind);
    return PublicResult(opening->_nativeOwner->prepare(mutation));
}

- (Core3DPartBooleanValues *)describeAuthoredBooleanParameterOpening:
    (Core3DAuthoredParameterOpening *)opening expected:(Core3DSceneSnapshot *)expected {
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner
        || ![expected isKindOfClass:Core3DSceneSnapshot.class]) return nil;
    try {
        GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        const auto viewer = gl ? gl.viewer : nullptr;
        const Handle(OcctDocument) document = viewer
            ? viewer->getDocument() : Handle(OcctDocument)();
        Core3DSceneSnapshot *current = [self captureSceneSnapshot];
        if (!viewer || !opening->_nativeOwner->belongsTo(document)
            || opening->_nativeOwner->currentness()
                != core3d::authored_parameter::Currentness::Current
            || !SameExpectedScene(expected, current, opening.entityIdentifier)) return nil;
        core3d::part_boolean::AnalyticDefinition definition;
        return opening->_nativeOwner->describeBoolean(definition)
            ? BooleanPublicValues(definition) : nil;
    } catch (...) { return nil; }
}

- (BOOL)cancelAuthoredParameterOpening:(Core3DAuthoredParameterOpening *)opening {
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner) return NO;
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    return opening->_nativeOwner->belongsTo(document) && opening->_nativeOwner->cancel();
}

#if DEBUG
- (BOOL)debugReportNextAuthoredParameterCloseUnproven:
    (Core3DAuthoredParameterOpening *)opening {
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner) return NO;
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer
        ? viewer->getDocument() : Handle(OcctDocument)();
    return opening->_nativeOwner->belongsTo(document)
        && opening->_nativeOwner->debugReportNextCloseUnproven();
}

- (BOOL)debugArmAuthoredBooleanStageFault:(Core3DAuthoredParameterOpening *)opening
    site:(Core3DDebugAuthoredBooleanFaultSite)site
    delivery:(Core3DDebugAuthoredBooleanFaultDelivery)delivery {
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner) return NO;
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer
        ? viewer->getDocument() : Handle(OcctDocument)();
    namespace abd = core3d::authored_parameter::debug;
    return opening->_nativeOwner->belongsTo(document)
        && opening->_nativeOwner->debugArmAuthoredBooleanStageFault(
            static_cast<abd::AuthoredBooleanFaultSite>(site),
            static_cast<abd::AuthoredBooleanFaultDelivery>(delivery));
}

- (Core3DDebugAuthoredBooleanFaultReceipt *)debugAuthoredBooleanStageFaultReceipt:
    (Core3DAuthoredParameterOpening *)opening {
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner) return nil;
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer
        ? viewer->getDocument() : Handle(OcctDocument)();
    if (!opening->_nativeOwner->belongsTo(document)) return nil;
    core3d::authored_parameter::debug::AuthoredBooleanFaultReceipt native;
    if (!opening->_nativeOwner->debugAuthoredBooleanStageFaultReceipt(native)) return nil;
    return [[Core3DDebugAuthoredBooleanFaultReceipt alloc]
        initWithSite:static_cast<Core3DDebugAuthoredBooleanFaultSite>(native.site)
        delivery:static_cast<Core3DDebugAuthoredBooleanFaultDelivery>(native.delivery)
        hitCount:native.hits];
}

- (Core3DDebugAuthoredBooleanObservation *)debugObserveAuthoredBooleanStateForEntityIdentifier:
    (NSString *)entityIdentifier {
    if (![NSThread isMainThread] || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128) return nil;
    try {
        GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)self.glController : nil;
        const auto viewer = gl ? gl.viewer : nullptr;
        const Handle(OcctDocument) document = viewer
            ? viewer->getDocument() : Handle(OcctDocument)();
        if (!viewer || document.IsNull()) return nil;
        const TDF_Label ownerLabel = ExactRoot(document, entityIdentifier.UTF8String ?: "");
        if (ownerLabel.IsNull()) return nil;
        core3d::authored_parameter::debug::AuthoredBooleanObservation native;
        if (!document->DebugObserveAuthoredBooleanState(ownerLabel, native)) return nil;
        NSMutableArray<Core3DDebugTreatmentNamingVersionRow *> *rows =
            [NSMutableArray arrayWithCapacity:native.naming.size()];
        for (const auto& row : native.naming) {
            NSString *entry = [[NSString alloc] initWithBytes:row.labelEntry.data()
                length:row.labelEntry.size() encoding:NSUTF8StringEncoding];
            if (!entry) return nil;
            [rows addObject:[[Core3DDebugTreatmentNamingVersionRow alloc]
                initWithLabelEntry:entry version:row.version]];
        }
        return [[Core3DDebugAuthoredBooleanObservation alloc]
            initWithNamingVersions:rows
            availableUndos:native.availableUndos availableRedos:native.availableRedos
            openCommand:native.openCommand pendingCompanion:native.pendingCompanion
            pendingAfterCaptured:native.pendingAfterCaptured
            finalizedCompanions:native.finalizedCompanions];
    } catch (...) { return nil; }
}
#endif

- (Core3DAuthoredParameterOperation *)applyAuthoredLoftParameterOpening:
    (Core3DAuthoredParameterOpening *)opening
    stationIdentifier:(uint32_t)stationIdentifier
    field:(Core3DAuthoredLoftField)field
    definition:(Core3DRectangularLoftDefinition *)definition
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DAuthoredParameterApplyResult *))completion {
    const auto settle = ^Core3DAuthoredParameterOperation *(
        Core3DAuthoredParameterApplyOutcome outcome, NSString *code, NSNumber *delta) {
        auto operation = [[Core3DAuthoredParameterOperation alloc]
            initWithWork:std::shared_ptr<core3d::retained_edge_treatment::r2::Work>()
            completion:completion];
        operation->_settled = YES;
        if (completion) completion([[Core3DAuthoredParameterApplyResult alloc]
            initWithOutcome:outcome refusalCode:code measuredUndoDelta:delta]);
        operation->_completion = nil;
        return operation;
    };
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner
        || ![definition isKindOfClass:Core3DRectangularLoftDefinition.class]
        || ![expected isKindOfClass:Core3DSceneSnapshot.class]
        || field < Core3DAuthoredLoftFieldStationZ
        || field > Core3DAuthoredLoftFieldFrameSignedScale)
        return settle(Core3DAuthoredParameterApplyOutcomeRefused,
                      @"authored.loft.malformed", @0);
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    if (!viewer || !opening->_nativeOwner->belongsTo(document))
        return settle(Core3DAuthoredParameterApplyOutcomeRefused,
                      @"authored.foreign-opening", @0);

    core3d::retained_edge_treatment::r2::Edit mutation;
    const auto authoredStatus = opening->_nativeOwner->prepareLoft(stationIdentifier,
        NativeLoftField(field), [definition nativeDefinition], mutation);
    if (authoredStatus == core3d::authored_loft::Status::Unchanged) {
        opening->_nativeOwner->cancel();
        return settle(Core3DAuthoredParameterApplyOutcomeUnchanged,
                      LoftStatusCode(authoredStatus), @0);
    }
    if (authoredStatus != core3d::authored_loft::Status::Prepared)
        return settle(Core3DAuthoredParameterApplyOutcomeRefused,
                      LoftStatusCode(authoredStatus), @0);

    const CGSize size = self.viewportDrawableSize;
    if (!std::isfinite(size.width) || !std::isfinite(size.height)
        || size.width < 1 || size.height < 1
        || size.width > UINT32_MAX || size.height > UINT32_MAX)
        return settle(Core3DAuthoredParameterApplyOutcomeRefused,
                      @"authored.stale-opening", @0);
    core3d::ObjectFrameIdentity identity;
    identity.entityIdentifier = opening.entityIdentifier.UTF8String ?: "";
    identity.publicationSourceIdentifier = expected.publicationSourceIdentifier.UTF8String ?: "";
    identity.documentGeneration = expected.revisions.documentGeneration;
    identity.modelRevision = expected.revisions.modelRevision;
    core3d::retained_edge_treatment::Refusal refusal;
    auto work = viewer->prepareEdgeTreatmentEditR2(
        opening->_nativeOwner->retainedSnapshot(), mutation, identity,
        expected.revisions.presentationRevision, std::uint32_t(size.width),
        std::uint32_t(size.height), refusal);
    opening->_nativeOwner->cancel();
    if (!work)
        return settle(Core3DAuthoredParameterApplyOutcomeRefused,
            [NSString stringWithUTF8String:
                core3d::retained_edge_treatment::RefusalCode(refusal)], @0);

    auto operation = [[Core3DAuthoredParameterOperation alloc]
        initWithWork:work completion:completion];
    auto geometry = core3d::Core3DViewer::edgeTreatmentGeometryR2(work);
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{
        core3d::retained_edge_treatment::Refusal buildRefusal;
        auto built = core3d::Core3DViewer::buildEdgeTreatmentR2(geometry, buildRefusal);
        dispatch_async(dispatch_get_main_queue(), ^{
            if (operation->_settled) return;
            operation->_settled = YES;
            core3d::retained_edge_treatment::CommitResult result;
            if (built) result = viewer->commitEdgeTreatmentR2(work, built);
            else result.refusal = buildRefusal;
            Core3DAuthoredParameterApplyOutcome outcome;
            using Outcome = core3d::retained_edge_treatment::CommitOutcome;
            switch (result.outcome) {
                case Outcome::Committed:
                    outcome = Core3DAuthoredParameterApplyOutcomeCommitted; break;
                case Outcome::Unchanged:
                    outcome = Core3DAuthoredParameterApplyOutcomeUnchanged; break;
                case Outcome::Cancelled:
                    outcome = Core3DAuthoredParameterApplyOutcomeCancelled; break;
                case Outcome::OutcomeUnknown:
                    outcome = Core3DAuthoredParameterApplyOutcomeOutcomeUnknown; break;
                case Outcome::Refused:
                case Outcome::Busy:
                    outcome = Core3DAuthoredParameterApplyOutcomeRefused; break;
            }
            NSNumber *delta = result.measuredUndoDelta
                ? @(*result.measuredUndoDelta) : nil;
            auto answer = [[Core3DAuthoredParameterApplyResult alloc]
                initWithOutcome:outcome
                refusalCode:[NSString stringWithUTF8String:
                    core3d::retained_edge_treatment::RefusalCode(result.refusal)]
                measuredUndoDelta:delta];
            auto callback = operation->_completion;
            operation->_completion = nil;
            operation->_work.reset();
            if (callback) callback(answer);
        });
    });
    return operation;
}

- (Core3DAuthoredParameterOperation *)applyAuthoredBooleanParameterOpening:
    (Core3DAuthoredParameterOpening *)opening
    mutationKind:(Core3DAuthoredParameterMutationKind)mutationKind
    inputIndex:(NSUInteger)inputIndex
    values:(Core3DPartBooleanValues *)values
    expected:(Core3DSceneSnapshot *)expected
    completion:(void(^)(Core3DAuthoredParameterApplyResult *))completion {
    auto operation = [[Core3DAuthoredParameterOperation alloc]
        initWithWork:std::shared_ptr<core3d::retained_edge_treatment::r2::Work>()
        completion:completion];
    const auto settle = ^(Core3DAuthoredParameterApplyOutcome outcome,
                          NSString *code, NSNumber *delta) {
        operation->_settled = YES;
        auto callback = operation->_completion;
        operation->_completion = nil;
        if (callback) callback([[Core3DAuthoredParameterApplyResult alloc]
            initWithOutcome:outcome refusalCode:code measuredUndoDelta:delta]);
    };
    if (![NSThread isMainThread]
        || ![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner || inputIndex > 1
        || ![expected isKindOfClass:Core3DSceneSnapshot.class]
        || (mutationKind != Core3DAuthoredParameterMutationKindBooleanAnalyticInput
            && mutationKind != Core3DAuthoredParameterMutationKindBooleanOperation
            && mutationKind != Core3DAuthoredParameterMutationKindBooleanPlacement)) {
        settle(Core3DAuthoredParameterApplyOutcomeRefused,
               @"authored.boolean.malformed", @0);
        return operation;
    }
    GLViewController *gl = [self.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)self.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer ? viewer->getDocument() : Handle(OcctDocument)();
    Core3DSceneSnapshot *current = [self captureSceneSnapshot];
    if (!viewer || !opening->_nativeOwner->belongsTo(document)
        || !SameExpectedScene(expected, current, opening.entityIdentifier)) {
        settle(Core3DAuthoredParameterApplyOutcomeRefused,
               @"authored.stale-opening", @0);
        return operation;
    }
    core3d::part_boolean::AnalyticDefinition original, requested;
    if (!opening->_nativeOwner->describeBoolean(original)
        || !BooleanRequest(original, values, requested)) {
        settle(Core3DAuthoredParameterApplyOutcomeRefused,
               @"authored.boolean.malformed", @0);
        return operation;
    }
    const auto result = opening->_nativeOwner->applyBoolean(
        static_cast<core3d::authored_parameter::Capability>(mutationKind),
        inputIndex, requested);
    Core3DAuthoredParameterApplyOutcome outcome;
    using NativeOutcome = core3d::authored_parameter::ApplyOutcome;
    if (result.outcome == NativeOutcome::OutcomeUnknown) {
        // A2-N3: the transaction's terminal state is unproven and the
        // document-owned recovery record is pending. Retain this exact
        // issuing owner on the host, bound to the exact native document and
        // data identities, so the production continuation can settle the
        // retained native record after the opening wrapper is released; the
        // issuer is superseded by the next fresh opening and cannot act on a
        // replaced document with matching textual identifiers.
        const Handle(TDocStd_Document) ocafDocument = document->Document();
        [Core3DAuthoredParameterRecoveryIssuers() setObject:
            [[Core3DAuthoredParameterRecoveryIssuer alloc]
                initWithOwner:opening->_nativeOwner
                ocafDocument:ocafDocument.IsNull() ? nullptr : ocafDocument.get()
                ocafData:ocafDocument.IsNull() ? nullptr
                    : ocafDocument->GetData().get()]
            forKey:self];
    }
    switch (result.outcome) {
        case NativeOutcome::Committed:
            outcome = Core3DAuthoredParameterApplyOutcomeCommitted; break;
        case NativeOutcome::Unchanged:
            outcome = Core3DAuthoredParameterApplyOutcomeUnchanged; break;
        case NativeOutcome::Cancelled:
            outcome = Core3DAuthoredParameterApplyOutcomeCancelled; break;
        case NativeOutcome::OutcomeUnknown:
            outcome = Core3DAuthoredParameterApplyOutcomeOutcomeUnknown; break;
        case NativeOutcome::Refused:
            outcome = Core3DAuthoredParameterApplyOutcomeRefused; break;
    }
    NSNumber *delta = result.measuredUndoDelta ? @(*result.measuredUndoDelta) : nil;
    settle(outcome, [NSString stringWithUTF8String:result.code.c_str()], delta);
    return operation;
}
@end

#if DEBUG
extern "C" void *Core3DDebugA1AnalyticRotationOracleProbe(int32_t scenario) {
    return (__bridge_retained void *)RotationOracleProbe(scenario);
}
extern "C" void *Core3DDebugA1AnalyticRotationCountedProbe(int32_t scenario) {
    return (__bridge_retained void *)RotationCountedProbe(scenario);
}
extern "C" bool Core3DDebugA2ArmAbortSettlementUnproven(void *openingObject) {
    Core3DAuthoredParameterOpening *opening =
        (__bridge Core3DAuthoredParameterOpening *)openingObject;
    return [opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        && opening->_nativeOwner
        && opening->_nativeOwner->debugArmNextAbortSettlementUnproven();
}
extern "C" bool Core3DDebugA2ArmRecoveryObservationFailure(void *openingObject) {
    Core3DAuthoredParameterOpening *opening =
        (__bridge Core3DAuthoredParameterOpening *)openingObject;
    return [opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        && opening->_nativeOwner
        && opening->_nativeOwner->debugArmNextRecoveryObservationFailure();
}
extern "C" void *Core3DDebugA2AuthoredBooleanRecoveryProbe(
    void *openingObject, void *hostObject) {
    Core3DAuthoredParameterOpening *opening =
        (__bridge Core3DAuthoredParameterOpening *)openingObject;
    Core3DViewController *host = (__bridge Core3DViewController *)hostObject;
    if (![opening isKindOfClass:Core3DAuthoredParameterOpening.class]
        || !opening->_nativeOwner || ![host isKindOfClass:Core3DViewController.class]) {
        return (__bridge_retained void *)@{
            @"outcome": @3, @"code": @"authored.recovery-not-owner"
        };
    }
    GLViewController *gl = [host.glController isKindOfClass:GLViewController.class]
        ? (GLViewController *)host.glController : nil;
    const auto viewer = gl ? gl.viewer : nullptr;
    const Handle(OcctDocument) document = viewer
        ? viewer->getDocument() : Handle(OcctDocument)();
    if (!opening->_nativeOwner->belongsTo(document)) {
        return (__bridge_retained void *)@{
            @"outcome": @3, @"code": @"authored.recovery-not-owner"
        };
    }
    const auto result = opening->_nativeOwner->reconcileBoolean();
    NSMutableDictionary *answer = [@{
        @"outcome": @(static_cast<NSInteger>(result.outcome)),
        @"code": [NSString stringWithUTF8String:result.code.c_str()],
        @"reconcileCalls": @(result.reconcileCalls),
        @"observationCalls": @(result.observationCalls),
        @"observationFaultHits": @(result.observationFaultHits),
        @"publicationCalls": @(result.publicationCalls),
        @"recoveryCommandCalls": @(result.recoveryCommandCalls),
    } mutableCopy];
    if (result.measuredUndoDelta)
        answer[@"measuredUndoDelta"] = @(*result.measuredUndoDelta);
    return (__bridge_retained void *)answer;
}

namespace {
core3d::retained_recipe::UUID I16UUID(std::uint8_t seed) {
    core3d::retained_recipe::UUID value{};
    for (std::size_t index = 0; index < value.size(); ++index)
        value[index] = std::uint8_t(seed + index);
    return value;
}

core3d::retained_recipe::Digest I16Digest(std::uint8_t seed) {
    core3d::retained_recipe::Digest value{};
    for (std::size_t index = 0; index < value.size(); ++index)
        value[index] = std::uint8_t(seed + index);
    return value;
}

core3d::part_boolean::SignatureCommitments I16Commitments(std::uint8_t seed) {
    using core3d::part_boolean::SignatureCommitments;
    return SignatureCommitments{I16Digest(seed), I16Digest(std::uint8_t(seed + 1)),
        I16Digest(std::uint8_t(seed + 2)), I16Digest(std::uint8_t(seed + 3)),
        I16Digest(std::uint8_t(seed + 4))};
}

core3d::part_boolean::OriginalsDefinition I16Fixture(double unit) {
    namespace pb = core3d::part_boolean;
    pb::OriginalsDefinition value;
    value.operation = pb::Operation::Subtract;
    value.originals.creationOperation = value.operation;
    value.owner = {I16UUID(1), I16UUID(21), I16UUID(41)};
    value.booleanNode = I16UUID(61); value.feature = I16UUID(81);
    value.carrierMetersPerUnit = unit;

    pb::ScalarMaterialRow material;
    material.labelEntry = "0:1:4"; material.faceCulling = 2;
    material.alphaMode = 1; material.alphaCutoff = 0.5;
    material.pbrDefined = true; material.commonDefined = true;
    material.pbr = {{1, 0.25, 0.5, 1, 0.1, 0.2, 0.3, 0.4, 0.6, 1.45}};
    material.common = {{0.1,0.1,0.1, 0.5,0.4,0.3, 0.8,0.8,0.8,
                        0.2,0.1,0.0, 0.75,0.125}};
    pb::GroupRow group; group.identifier = "group-stable-1";
    group.name.attributePresent = true; group.name.codeUnits = {'G','r','o','u','p'};
    group.originPresent = true; group.origin = {{1, 2, 3}};
    group.members.push_back({value.owner.document, I16UUID(101), I16UUID(121)});
    pb::LayerRow layer; layer.labelEntry = "0:2:7";
    layer.name.attributePresent = true; layer.name.codeUnits = {'L','a','y','e','r'};
    layer.invisibleAttributePresent = true;
    value.originals.materials = {material}; value.originals.groups = {group};
    value.originals.layers = {layer};
    value.effective.materials = {material}; value.effective.groups = {group};
    value.effective.layers = {layer};

    for (std::size_t index = 0; index < 2; ++index) {
        auto& original = value.originals.inputs[index];
        auto& effective = value.effective.inputs[index];
        original.role = effective.role = index == 0
            ? pb::RecipeInputRole::Left : pb::RecipeInputRole::Right;
        original.sourceNode = effective.sourceNode = I16UUID(std::uint8_t(141 + index * 20));
        original.sourceLocalID = index + 1; original.shapeSlot = std::uint32_t(index);
        original.original = {value.owner.document, I16UUID(std::uint8_t(181 + index * 20)),
            I16UUID(std::uint8_t(201 + index * 20)), I16UUID(std::uint8_t(221 + index * 20))};
        original.recipeKind = pb::RecipeInputFamily::Profile;
        original.recipeSchema = 4; original.recipeBytes = {0x53,0x59,0x4c,0x56,1,std::uint8_t(index)};
        original.sourceMetersPerUnit = unit; original.carrierMetersPerUnit = unit;
        original.projection.dimensions = {{40 + double(index), 30, 20}};
        original.projection.translation = {{index == 0 ? -0.0 : 12.0, 2, 3}};
        original.commitments = effective.commitments = I16Commitments(std::uint8_t(11 + index * 10));
        effective.originalSourceFeature = original.original.sourceFeature;
        effective.geometry.dimensions = original.projection.dimensions;
        effective.geometry.translation = original.projection.translation;
        effective.geometry.rotationXYZW = original.projection.rotationXYZW;
        effective.sourceMetersPerUnit = unit;
        auto configureMetadata = [index](pb::CompleteMetadata& metadata) {
            metadata.name.attributePresent = index != 0;
            metadata.name.codeUnits = {};
            metadata.occurrence.presenceMask = index == 0 ? 0 : 0xff;
            if (index != 0) {
                metadata.occurrence.values = {{-0.0, 0, 0, -0.0, 0, 0, 1, 1}};
            }
            metadata.appearance.legacyMaterialPresent = true;
            metadata.appearance.legacyMaterial = 7;
            metadata.appearance.localPBRMarkerPresent = true;
            metadata.appearance.material = 0;
            metadata.groups = {0}; metadata.invisibleAttributePresent = index != 0;
            metadata.layerLinkPresent = true; metadata.layers = {0};
        };
        configureMetadata(original.metadata); configureMetadata(effective.metadata);
    }
    return value;
}
} // namespace

extern "C" void *Core3DDebugI16OriginalsCodecProbe(double unit) {
    namespace pb = core3d::part_boolean;
    namespace cr = core3d::composite_recipe;
    NSMutableDictionary<NSString *, NSNumber *> *checks = [NSMutableDictionary dictionary];
    const auto put = [&](NSString *key, bool value) { checks[key] = @(value); };
    try {
        const pb::OriginalsDefinition fixture = I16Fixture(unit);
        std::vector<std::uint8_t> bytes, exact;
        pb::OriginalsDefinition decoded;
        const bool encoded = pb::EncodeOriginals(fixture, bytes);
        const bool decodedOK = encoded && pb::DecodeOriginals(bytes, decoded)
            && pb::EncodeOriginals(decoded, exact) && exact == bytes;
        put(@"canonical-roundtrip", decodedOK);
        put(@"bounded-16384", encoded && bytes.size() <= pb::MaximumPayloadBytes);
        put(@"both-unit-carrier-bits", decodedOK
            && pb::SameDoubleBits(decoded.carrierMetersPerUnit, unit));
        put(@"signed-zero-preserved", decodedOK
            && std::signbit(decoded.originals.inputs[0].projection.translation[0])
            && std::signbit(decoded.originals.inputs[1].metadata.occurrence.values[0]));
        put(@"absent-versus-present-empty-name", decodedOK
            && !decoded.originals.inputs[0].metadata.name.attributePresent
            && decoded.originals.inputs[1].metadata.name.attributePresent
            && decoded.originals.inputs[1].metadata.name.codeUnits.empty());
        put(@"material-sharing-groups-layers", decodedOK
            && decoded.originals.materials.size() == 1
            && decoded.originals.inputs[0].metadata.appearance.material == 0
            && decoded.originals.inputs[1].metadata.appearance.material == 0
            && decoded.originals.groups.size() == 1 && decoded.originals.layers.size() == 1);
        put(@"typed-original-effective-separation", decodedOK
            && pb::correspondence::EffectiveMatchesOriginalAtCreation(
                decoded.originals.inputs[0].projection, decoded.effective.inputs[0].geometry));

        std::vector<std::uint8_t> originalsBefore, originalsAfter;
        pb::EncodeOriginalStateBlock(decoded.originals, originalsBefore);
        decoded.effective.inputs[0].geometry.dimensions[0] += 1;
        pb::EncodeOriginalStateBlock(decoded.originals, originalsAfter);
        put(@"effective-edit-keeps-frozen-block", originalsBefore == originalsAfter);

        auto malformed = bytes; if (malformed.size() > 4) malformed[4] = 9;
        pb::OriginalsDefinition refused;
        put(@"unknown-version-refused", !pb::DecodeOriginals(malformed, refused));
        malformed = bytes; if (!malformed.empty()) malformed.pop_back();
        put(@"truncation-refused", !pb::DecodeOriginals(malformed, refused));
        malformed = bytes; malformed.push_back(0);
        put(@"extra-byte-refused", !pb::DecodeOriginals(malformed, refused));
        malformed = bytes; if (!malformed.empty()) malformed.back() ^= 1;
        put(@"bad-digest-refused", !pb::DecodeOriginals(malformed, refused));
        malformed.assign(pb::MaximumPayloadBytes + 1, 0);
        put(@"aggregate-overflow-refused", !pb::DecodeOriginals(malformed, refused));

        auto invalid = fixture; invalid.originals.schema = 2;
        put(@"unknown-schema-refused", !pb::EncodeOriginals(invalid, exact));
        invalid = fixture; invalid.effective.inputs[0].family = pb::InputFamily::ShellProfile;
        put(@"unknown-family-tag-refused", !pb::EncodeOriginals(invalid, exact));
        invalid = fixture; invalid.originals.inputs[0].recipeBytes.assign(
            pb::MaximumRecipeBytes + 1, 1);
        put(@"oversized-length-refused", !pb::EncodeOriginals(invalid, exact));
        invalid = fixture; invalid.originals.inputs[0].metadata.groups = {0, 0};
        put(@"duplicate-reference-refused", !pb::EncodeOriginals(invalid, exact));
        invalid = fixture; invalid.originals.inputs[0].metadata.layers = {7};
        put(@"dangling-reference-refused", !pb::EncodeOriginals(invalid, exact));
        invalid = fixture; invalid.originals.inputs[0].original.document = I16UUID(250);
        put(@"owner-binding-refused", !pb::EncodeOriginals(invalid, exact));
        invalid = fixture; invalid.originals.inputs[0].shapeSlot = 1;
        put(@"duplicate-slot-refused", !pb::EncodeOriginals(invalid, exact));
        invalid = fixture; invalid.effective.inputs[0].role = pb::RecipeInputRole::Right;
        put(@"role-binding-refused", !pb::EncodeOriginals(invalid, exact));
        invalid = fixture; invalid.originals.materials.push_back(invalid.originals.materials[0]);
        invalid.originals.materials.back().ordinal = 1;
        put(@"duplicate-table-provenance-refused", !pb::EncodeOriginals(invalid, exact));

        const auto& registry = core3d::retained_feature::ProductionRegistry();
        const auto* descriptor = registry.find({cr::PartBooleanFeatureKind,
                                                cr::PartBooleanOriginalsFeatureCodec});
        put(@"production-descriptor-readable", descriptor != nullptr && registry.valid());
        put(@"execution-callbacks-empty", descriptor && !descriptor->execution.installed()
            && !descriptor->execution.buildDetached && !descriptor->execution.proveFamily
            && !descriptor->execution.verifyFixedPoint
            && !descriptor->execution.editorRouteInstalled
            && !descriptor->execution.dependencyRouteInstalled);
        put(@"unknown-registry-key-refused", registry.find(
            {cr::PartBooleanFeatureKind, UINT32_MAX}) == nullptr);
        put(@"legacy-availability-not-recorded",
            pb::AvailabilityForCodec(pb::LegacyCodecVersion)
                == pb::OriginalsAvailability::LegacyNotRecorded);
        put(@"v4-availability-complete",
            pb::AvailabilityForCodec(pb::OriginalsCodecVersion)
                == pb::OriginalsAvailability::Complete);

        cr::FeatureNode feature;
        feature.node = fixture.booleanNode; feature.feature = fixture.feature;
        feature.kind = cr::PartBooleanFeatureKind;
        feature.codecVersion = cr::PartBooleanOriginalsFeatureCodec;
        feature.parameters = bytes;
        std::vector<cr::Node> sources;
        for (std::size_t index = 0; index < 2; ++index) {
            const auto& original = fixture.originals.inputs[index];
            cr::SourceNode source; source.node = original.sourceNode;
            source.localID = original.sourceLocalID; source.original = original.original;
            source.recipe.kind = cr::RecipeKind(std::uint8_t(original.recipeKind));
            source.recipe.schema = original.recipeSchema; source.recipe.bytes = original.recipeBytes;
            source.inputToCarrier.sourceMetersPerUnit = original.sourceMetersPerUnit;
            source.inputToCarrier.carrierMetersPerUnit = original.carrierMetersPerUnit;
            source.inputToCarrier.matrix = original.sourceRootToCarrier;
            source.commitments = {original.commitments.geometry, original.commitments.recipe,
                original.commitments.placement, original.commitments.material,
                original.commitments.groups};
            source.shapeSlot = original.shapeSlot; sources.push_back({source});
            feature.inputs.push_back(source.node);
        }
        put(@"canonical-production-binding", cr::CanonicalOriginalsFeature(feature, sources));
        std::get<cr::SourceNode>(sources[0].value).shapeSlot = 1;
        put(@"mismatched-source-binding-refused",
            !cr::CanonicalOriginalsFeature(feature, sources));
    } catch (...) { checks[@"probe-exception-free"] = @NO; }
    return (__bridge_retained void *)[checks copy];
}
#endif
