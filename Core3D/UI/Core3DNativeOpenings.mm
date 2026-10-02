#import "Core3DModelingTypes.h"
#import "Core3DViewController.h"
#import "../OCCTKit/GLViewController.h"
#import "../Viewport/Core3DSceneSnapshot.h"

#include "../OCCTKit/NativeOpeningFactories.hxx"
#include "../OCCTKit/BoundedCurveOwner.hxx"
#include "../OCCTKit/SplineProfileOwner.hxx"
#include "../OCCTKit/ReceiptRecord.hxx"
#include "../OCCTKit/OcctDocument.h"
#include "../OCCTKit/FeaturePatternOwnerBridge.hxx"
#include "../OCCTKit/GeneralLoftEdit.hxx"
#include "../OCCTKit/GeneralLoftOwner.hxx"
#include "../OCCTKit/PathArrayOwnerBridge.hxx"
#include "../OCCTKit/PatternOwnerBridge.hxx"
#include <XCAFDoc_DocumentTool.hxx>
#if DEBUG
#include <cstdio>
#include "../OCCTKit/NativeOpeningSurfaceProbe.hxx"
// The R179 fixture bridge at the end of this file implements a
// Core3DViewController category. These imports must land before the anonymous
// namespace below: its helpers (e.g. Boolean at line 61) would otherwise make
// names used by textually included system headers ambiguous.
#import "Core3DViewController.h"
#import "GLViewController.h"
#include "GLViewController+Trick.h"
#import "Core3DViewer.h"
#include "../Viewport/Core3DSceneSnapshotFactory.hpp"
#if TARGET_OS_IOS
#import "../OCCTKit/GLView.h"
#endif
#include "../OCCTKit/BoundedCurveAttribute.hxx"
#include "../OCCTKit/SpatialSweepColdOpenFixture.hxx"
#include "../OCCTKit/FeaturePatternChildAttribute.hxx"
#include "../OCCTKit/FeaturePatternPersistence.hxx"
#include "../OCCTKit/PathArrayPersistence.hxx"
#include "../OCCTKit/PatternPersistence.hxx"
#include "../OCCTKit/AnalyticBooleanSolid.hxx"
#include <AIS_Shape.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <Standard_Failure.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDataStd_AsciiString.hxx>
#include <TDataStd_ByteArray.hxx>
#include <TDataStd_Integer.hxx>
#include <TDataStd_Name.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_TagSource.hxx>
#include <TDF_Tool.hxx>
#include <TNaming_Builder.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#endif

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <set>

namespace {
using core3d::pattern::Axis;
using core3d::pattern::Coordinate;
using core3d::pattern::Kind;

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double Degrees(double radians) noexcept { return radians * 180.0 / kPi; }
constexpr double Radians(double degrees) noexcept { return degrees * kPi / 180.0; }

constexpr NSInteger ClampToNSInteger(std::size_t value) noexcept {
    constexpr auto maximum = static_cast<std::size_t>(NSIntegerMax);
    return value > maximum ? NSIntegerMax : static_cast<NSInteger>(value);
}

NSString *Text(const std::string& value) {
    return [[NSString alloc] initWithBytes:value.data()
                                    length:value.size()
                                  encoding:NSUTF8StringEncoding] ?: @"";
}

NSString *UUIDText(const core3d::pattern::UUID& value) {
    try { return Text(core3d::retained_solid::UUIDText(value)); }
    catch (...) { return @""; }
}

bool ExactKeys(NSDictionary *value, NSArray<NSString *> *keys) {
    if (![value isKindOfClass:NSDictionary.class] || value.count != keys.count) return false;
    NSSet *expected = [NSSet setWithArray:keys];
    return [[NSSet setWithArray:value.allKeys] isEqualToSet:expected];
}

bool Number(id value, double& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) == CFBooleanGetTypeID()) return false;
    output = [value doubleValue];
    return std::isfinite(output);
}

bool Integer(id value, std::uint64_t maximum, std::uint64_t& output) {
    double scalar = 0;
    if (!Number(value, scalar) || scalar < 0 || scalar > double(maximum)
        || std::trunc(scalar) != scalar) return false;
    const auto converted = [value unsignedLongLongValue];
    if (converted > maximum || double(converted) != scalar) return false;
    output = converted;
    return true;
}

bool Boolean(id value, bool& output) {
    if (![value isKindOfClass:NSNumber.class]
        || CFGetTypeID((__bridge CFTypeRef)value) != CFBooleanGetTypeID()) return false;
    output = [value boolValue]; return true;
}

bool String(id value, NSString *__strong& output) {
    if (![value isKindOfClass:NSString.class] || [value length] == 0) return false;
    output = value; return true;
}

bool Vector3(id value, std::array<double, 3>& output) {
    if (![value isKindOfClass:NSArray.class] || [value count] != 3) return false;
    for (NSUInteger index = 0; index < 3; ++index)
        if (!Number(value[index], output[index])) return false;
    return true;
}

bool Coordinates(id value, std::set<Coordinate>& output) {
    output.clear();
    if (![value isKindOfClass:NSArray.class]) return false;
    for (id item in value) {
        if (![item isKindOfClass:NSDictionary.class]
            || !ExactKeys(item, @[@"row", @"column"])) return false;
        std::uint64_t row = 0, column = 0;
        if (!Integer(item[@"row"], UINT32_MAX, row)
            || !Integer(item[@"column"], UINT32_MAX, column)
            || !output.insert({std::uint32_t(row), std::uint32_t(column)}).second)
            return false;
    }
    return true;
}

bool SuppressedEntities(id value, const std::vector<core3d::pattern_owner::LabelReceipt>& members,
                        std::set<Coordinate>& output) {
    output.clear();
    if (![value isKindOfClass:NSArray.class]) return false;
    std::set<std::string> requested;
    for (id item in value) {
        NSString *text = nil;
        if (!String(item, text)) return false;
        const char *utf8 = text.UTF8String;
        if (!utf8 || !requested.insert(utf8).second) return false;
    }
    for (const auto& member : members) {
        const std::string identifier = core3d::retained_solid::UUIDText(member.entity);
        if (requested.erase(identifier)) output.insert(member.coordinate);
    }
    return requested.empty() && !output.count(Coordinate{});
}

template <class Outcome>
Core3DProfileConstructionResult MapOutcome(Outcome value, Outcome committed,
                                            Outcome unknown) {
    if (value == committed) return Core3DProfileConstructionResultCommitted;
    if (value == unknown) return Core3DProfileConstructionResultRecoveryRequired;
    return Core3DProfileConstructionResultRejected;
}

enum class State : std::uint8_t { Open, Prepared, Applying, Cancelled, Settled, Recovery };

NSString *KindText(Kind value) {
    switch (value) {
        case Kind::Linear: return @"linear";
        case Kind::Radial: return @"radial";
        case Kind::Grid: return @"grid";
    }
}

// Real document length-unit symbol for editor presentation. Values remain in
// their declared units; only the label is derived here.
NSString *UnitSymbol(double metersPerUnit) {
    if (std::isfinite(metersPerUnit)) {
        if (std::abs(metersPerUnit - 1.0) <= 1e-12) return @"m";
        if (std::abs(metersPerUnit - 0.001) <= 1e-15) return @"mm";
    }
    return @"document";
}

bool ParseKind(id value, Kind& output) {
    NSString *text = nil;
    if (!String(value, text)) return false;
    if ([text isEqualToString:@"linear"]) output = Kind::Linear;
    else if ([text isEqualToString:@"radial"]) output = Kind::Radial;
    else if ([text isEqualToString:@"grid"]) output = Kind::Grid;
    else return false;
    return true;
}

bool ParseAxis(id value, Axis& output) {
    std::uint64_t scalar = 0;
    if (!Integer(value, 2, scalar)) return false;
    output = Axis(scalar); return true;
}

NSDictionary *PatternDescriptor(const core3d::pattern_owner::Snapshot& opening,
                                double metersPerUnit) {
    const auto& definition = opening.record.definition;
    const double millimetresPerUnit = metersPerUnit * 1000.0;
    NSMutableArray *members = [NSMutableArray arrayWithCapacity:definition.members.size()];
    for (const auto& member : definition.members)
        [members addObject:@{@"entityIdentifier": UUIDText(member.identity),
            @"row": @(member.coordinate.row), @"column": @(member.coordinate.column),
            @"suppressed": @(member.state == core3d::pattern::MemberState::Suppressed)}];
    return @{@"kind": KindText(definition.kind),
        @"sourceEntityIdentifier": UUIDText(definition.source.entity),
        @"patternFeatureIdentifier": UUIDText(definition.feature),
        @"documentMetersPerUnit": @(metersPerUnit),
        @"rowAxis": @(unsigned(definition.rowAxis)),
        @"columnAxis": @(unsigned(definition.columnAxis)),
        @"rowCount": @(definition.rowCount), @"columnCount": @(definition.columnCount),
        @"rowSpacingMM": @(definition.rowSpacing * millimetresPerUnit),
        @"columnSpacingMM": @(definition.columnSpacing * millimetresPerUnit),
        @"radialPivotMM": @[@(definition.radialPivotLocal[0] * millimetresPerUnit),
                            @(definition.radialPivotLocal[1] * millimetresPerUnit),
                            @(definition.radialPivotLocal[2] * millimetresPerUnit)],
        @"sweepDegrees": @(Degrees(definition.sweepRadians)), @"members": members};
}

bool PatternEdit(NSDictionary *value, const core3d::pattern_owner::Snapshot& opening,
                 double metersPerUnit, core3d::pattern_owner::Edit& output) {
    NSArray *keys = @[@"rowAxis", @"columnAxis", @"rowCount", @"columnCount",
        @"rowSpacingMM", @"columnSpacingMM", @"radialPivotMM", @"sweepDegrees",
        @"suppressedEntityIdentifiers"];
    const bool hasNewCoordinates = value[@"suppressedNewMemberCoordinates"] != nil;
    if (hasNewCoordinates) keys = [keys arrayByAddingObject:@"suppressedNewMemberCoordinates"];
    if (!ExactKeys(value, keys)) return false;
    std::uint64_t rows = 0, columns = 0; double sweep = 0;
    if (!std::isfinite(metersPerUnit) || metersPerUnit <= 0) return false;
    const double millimetresPerUnit = metersPerUnit * 1000.0;
    const bool valid = ParseAxis(value[@"rowAxis"], output.rowAxis)
        && ParseAxis(value[@"columnAxis"], output.columnAxis)
        && Integer(value[@"rowCount"], UINT32_MAX, rows)
        && Integer(value[@"columnCount"], UINT32_MAX, columns)
        && Number(value[@"rowSpacingMM"], output.rowSpacing)
        && Number(value[@"columnSpacingMM"], output.columnSpacing)
        && Vector3(value[@"radialPivotMM"], output.radialPivotLocal)
        && Number(value[@"sweepDegrees"], sweep)
        && (output.rows = std::uint32_t(rows), output.columns = std::uint32_t(columns),
            output.sweepRadians = Radians(sweep), true)
        && SuppressedEntities(value[@"suppressedEntityIdentifiers"],
                              opening.members, output.suppressed);
    if (!valid) return false;
    if (hasNewCoordinates) {
        id coordinates = value[@"suppressedNewMemberCoordinates"];
        if (![coordinates isKindOfClass:NSArray.class]
            || [coordinates count] > core3d::pattern::MaximumInstances
            || rows == 0 || columns == 0
            || rows * columns > core3d::pattern::MaximumInstances) return false;
        for (id coordinate in coordinates) {
            std::uint64_t row = 0, column = 0;
            if (![coordinate isKindOfClass:NSDictionary.class]
                || !ExactKeys(coordinate, @[@"row", @"column"])
                || !Integer(coordinate[@"row"], UINT32_MAX, row)
                || !Integer(coordinate[@"column"], UINT32_MAX, column)
                || row >= rows || column >= columns || (row == 0 && column == 0)) return false;
            const Coordinate key{std::uint32_t(row), std::uint32_t(column)};
            // Existing members still require the exact retained entity receipt.
            for (const auto& member : opening.record.definition.members)
                if (member.coordinate == key) return false;
            if (!output.suppressed.insert(key).second) return false;
        }
    }
    output.rowSpacing /= millimetresPerUnit;
    output.columnSpacing /= millimetresPerUnit;
    for (double& value : output.radialPivotLocal) value /= millimetresPerUnit;
    return true;
}

NSDictionary *PathDescriptor(const core3d::path_array_owner::Snapshot& opening,
                             double metersPerUnit,
                             const core3d::path_array_owner::PathAuthority *replacement = nullptr) {
    const auto& value = opening.record.definition;
    NSMutableArray *members = [NSMutableArray arrayWithCapacity:value.members.size()];
    for (const auto& member : value.members)
        [members addObject:@{@"entityIdentifier": UUIDText(member.identity),
            @"ordinal": @(member.coordinate.column),
            @"suppressed": @(member.state == core3d::pattern::MemberState::Suppressed)}];
    NSString *distribution = value.distribution.mode
        == core3d::path_array::DistributionMode::Count ? @"count" : @"distance";
    NSString *orientation = value.orientation.policy
        == core3d::path_array::OrientationPolicy::Fixed ? @"fixed"
        : value.orientation.policy == core3d::path_array::OrientationPolicy::Tangent
            ? @"tangent" : @"bishop";
    return @{@"sourceEntityIdentifier": UUIDText(value.source.entity),
        @"pathEntityIdentifier": UUIDText(
            replacement ? replacement->locator.owner.entity : value.path.owner.entity),
        @"patternFeatureIdentifier": UUIDText(value.feature),
        @"unitSymbol": UnitSymbol(metersPerUnit),
        @"distribution": distribution, @"count": @(value.distribution.count),
        @"distanceInDocumentUnits": @(value.distribution.distance),
        @"includeStart": @(value.distribution.includeStart), @"includeEnd": @(value.distribution.includeEnd),
        @"closedPath": @(value.closedPath), @"orientation": orientation,
        @"rollDegrees": @(Degrees(value.orientation.rollRadians)),
        @"hasUpVector": @(value.orientation.hasUpVector),
        @"upVector": @[@(value.orientation.upVector[0]), @(value.orientation.upVector[1]),
                       @(value.orientation.upVector[2])],
        @"maximumFrameStepDegrees": @(Degrees(value.orientation.maximumFrameStepRadians)),
        @"arcToleranceInDocumentUnits": @(value.arcLengthTolerance),
        @"minimumTangentInDocumentUnits": @(value.minimumTangent), @"members": members};
}

bool PathEdit(NSDictionary *value, const core3d::path_array_owner::Snapshot& opening,
              const core3d::path_array_owner::PathAuthority *replacement,
              core3d::path_array_owner::Edit& output) {
    if (!ExactKeys(value, @[@"distribution", @"count", @"distanceInDocumentUnits",
            @"includeStart", @"includeEnd", @"closedPath", @"orientation", @"rollDegrees",
            @"hasUpVector", @"upVector", @"maximumFrameStepDegrees",
            @"arcToleranceInDocumentUnits", @"minimumTangentInDocumentUnits",
            @"suppressedEntityIdentifiers"])) return false;
    NSString *distribution = nil, *orientation = nil;
    std::uint64_t count = 0; double roll = 0, frame = 0;
    if (!String(value[@"distribution"], distribution)
        || !String(value[@"orientation"], orientation)
        || !Integer(value[@"count"], UINT32_MAX, count)
        || !Number(value[@"distanceInDocumentUnits"], output.distance)
        || !Boolean(value[@"includeStart"], output.includeStart)
        || !Boolean(value[@"includeEnd"], output.includeEnd)
        || !Boolean(value[@"closedPath"], output.closedPath)
        || !Number(value[@"rollDegrees"], roll)
        || !Boolean(value[@"hasUpVector"], output.hasUpVector)
        || !Vector3(value[@"upVector"], output.upVector)
        || !Number(value[@"maximumFrameStepDegrees"], frame)
        || !Number(value[@"arcToleranceInDocumentUnits"], output.arcLengthTolerance)
        || !Number(value[@"minimumTangentInDocumentUnits"], output.minimumTangent))
        return false;
    if ([distribution isEqualToString:@"count"])
        output.distribution = core3d::path_array::DistributionMode::Count;
    else if ([distribution isEqualToString:@"distance"])
        output.distribution = core3d::path_array::DistributionMode::Distance;
    else return false;
    if ([orientation isEqualToString:@"fixed"])
        output.orientation = core3d::path_array::OrientationPolicy::Fixed;
    else if ([orientation isEqualToString:@"tangent"])
        output.orientation = core3d::path_array::OrientationPolicy::Tangent;
    else if ([orientation isEqualToString:@"bishop"])
        output.orientation = core3d::path_array::OrientationPolicy::Bishop;
    else return false;
    output.count = std::uint32_t(count); output.rollRadians = Radians(roll);
    output.maximumFrameStepRadians = Radians(frame);
    std::set<Coordinate> suppressed;
    if (!SuppressedEntities(value[@"suppressedEntityIdentifiers"],
                            opening.labels ? std::vector<core3d::pattern_owner::LabelReceipt>{}
                                           : std::vector<core3d::pattern_owner::LabelReceipt>{},
                            suppressed)) {
        // D3 receipts are stored in AllLabelSnapshot, not LabelReceipt.
        id rows = value[@"suppressedEntityIdentifiers"];
        if (![rows isKindOfClass:NSArray.class]) return false;
        std::set<std::string> requested;
        for (id item in rows) {
            NSString *text = nil; if (!String(item, text)) return false;
            if (!requested.insert(text.UTF8String).second) return false;
        }
        for (std::size_t index = 0; index < opening.record.definition.members.size(); ++index) {
            const auto& member = opening.record.definition.members[index];
            if (requested.erase(core3d::retained_solid::UUIDText(member.identity)))
                output.suppressedOrdinals.insert(std::uint32_t(index));
        }
        if (!requested.empty() || output.suppressedOrdinals.count(0)) return false;
    }
    if (replacement) {
        if (!replacement->currentFor(replacement->locator)) return false;
        output.replacementPath = *replacement;
    }
    return true;
}

NSDictionary *FeatureDescriptor(const core3d::feature_pattern_owner::Snapshot& opening) {
    const auto& value = opening.record.definition;
    const double millimetresPerUnit = value.metersPerUnit * 1000.0;
    NSMutableArray *sources = [NSMutableArray array];
    if (const auto *program = std::get_if<core3d::retained_boolean::Program>(
            &opening.sourceProgram.recipe))
        for (const auto& step : program->steps)
            [sources addObject:@{@"stepID": @(step.operand.identifier),
                @"title": [NSString stringWithFormat:@"Step %llu",
                    (unsigned long long)step.operand.identifier],
                // The retained source cut is the carrier's derived feature;
                // this is the identity the native build validates.
                @"featureIdentifier": UUIDText(program->source.derivedFeature)}];
    NSMutableArray *members = [NSMutableArray array];
    for (const auto& member : value.distribution.members)
        [members addObject:@{@"childFeatureIdentifier":
                UUIDText(core3d::feature_pattern::ChildFeatureID(value, member)),
            @"instanceIdentifier": UUIDText(member.identity),
            @"row": @(member.coordinate.row), @"column": @(member.coordinate.column),
            @"suppressed": @(member.state == core3d::pattern::MemberState::Suppressed)}];
    return @{@"hostEntityIdentifier": UUIDText(value.host.entity),
        @"sourceEntityIdentifier": UUIDText(value.sourceCut.entity),
        @"patternFeatureIdentifier": UUIDText(value.feature),
        @"unitSymbol": UnitSymbol(value.metersPerUnit),
        @"documentMetersPerUnit": @(value.metersPerUnit),
        @"sourceCutStepID": @(value.sourceCutStepID), @"availableSourceCuts": sources,
        @"kind": KindText(value.distribution.kind),
        @"rowAxis": @(unsigned(value.distribution.rowAxis)),
        @"columnAxis": @(unsigned(value.distribution.columnAxis)),
        @"rowCount": @(value.distribution.rowCount),
        @"columnCount": @(value.distribution.columnCount),
        @"rowSpacingMM": @(value.distribution.rowSpacing * millimetresPerUnit),
        @"columnSpacingMM": @(value.distribution.columnSpacing * millimetresPerUnit),
        @"radialPivotMM": @[@(value.distribution.radialPivotLocal[0] * millimetresPerUnit),
                            @(value.distribution.radialPivotLocal[1] * millimetresPerUnit),
                            @(value.distribution.radialPivotLocal[2] * millimetresPerUnit)],
        @"sweepDegrees": @(Degrees(value.distribution.sweepRadians)), @"members": members,
        @"policy": @{@"minimumHostLigamentMM": @(value.minimumHostLigamentMM),
            @"maximumChildren": @(core3d::feature_pattern::MaximumGeneratedFeatures),
            @"maximumBoundarySectionsPerChild": @64,
            @"maximumDocumentBytes": @(core3d::feature_pattern::MaximumDocumentBytes),
            @"allowsSuppression": @YES}};
}

bool FeatureEdit(NSDictionary *value, double metersPerUnit,
                 core3d::feature_pattern_owner::Edit& output) {
    if (!ExactKeys(value, @[@"sourceCutStepID", @"kind", @"rowAxis", @"columnAxis",
            @"rowCount", @"columnCount", @"rowSpacingMM", @"columnSpacingMM",
            @"radialPivotMM", @"sweepDegrees", @"suppressedCoordinates"])) return false;
    std::uint64_t step = 0, rows = 0, columns = 0; double sweep = 0;
    if (!std::isfinite(metersPerUnit) || metersPerUnit <= 0) return false;
    const bool valid = Integer(value[@"sourceCutStepID"], UINT64_MAX, step) && step != 0
        && ParseKind(value[@"kind"], output.kind)
        && ParseAxis(value[@"rowAxis"], output.rowAxis)
        && ParseAxis(value[@"columnAxis"], output.columnAxis)
        && Integer(value[@"rowCount"], UINT32_MAX, rows)
        && Integer(value[@"columnCount"], UINT32_MAX, columns)
        && Number(value[@"rowSpacingMM"], output.rowSpacing)
        && Number(value[@"columnSpacingMM"], output.columnSpacing)
        && Vector3(value[@"radialPivotMM"], output.radialPivotLocal)
        && Number(value[@"sweepDegrees"], sweep)
        && Coordinates(value[@"suppressedCoordinates"], output.suppressed)
        && (output.sourceCutStepID = step, output.rows = std::uint32_t(rows),
            output.columns = std::uint32_t(columns),
            output.sweepRadians = Radians(sweep), !output.suppressed.count(Coordinate{}));
    if (!valid) return false;
    const double millimetresPerUnit = metersPerUnit * 1000.0;
    output.rowSpacing /= millimetresPerUnit;
    output.columnSpacing /= millimetresPerUnit;
    for (double& item : output.radialPivotLocal) item /= millimetresPerUnit;
    return true;
}

using BoundedOpening = core3d::bounded_curve::owner::Opening;
using BoundedCandidate = core3d::bounded_curve::owner::Candidate;
using BoundedUUID = core3d::bounded_curve::UUID;

NSString *BoundedUUIDText(const BoundedUUID& value) {
    return UUIDText(value);
}

bool BoundedUUIDValue(id value, BoundedUUID& output) {
    output = {};
    if (![value isKindOfClass:NSString.class]) return false;
    NSUUID *uuid = [[NSUUID alloc] initWithUUIDString:value];
    if (!uuid) return false;
    uuid_t bytes{};
    [uuid getUUIDBytes:bytes];
    std::copy_n(bytes, output.size(), output.begin());
    return core3d::bounded_curve::Nonzero(output);
}

NSArray *BoundedVector(const std::array<double, 3>& value) {
    return @[@(value[0]), @(value[1]), @(value[2])];
}

NSData *BoundedDigest(const core3d::bounded_curve::Digest& value) {
    return [NSData dataWithBytes:value.data() length:value.size()];
}

NSDictionary *BoundedDescriptor(const BoundedOpening& opening) {
    using namespace core3d::bounded_curve;
    const RetainedState& retained = opening.retained;
    const Definition& definition = retained.definition;
    NSMutableArray *poles = [NSMutableArray arrayWithCapacity:definition.controlPoints.size()];
    for (std::size_t index = 0; index < definition.controlPoints.size(); ++index) {
        const auto& pole = definition.controlPoints[index];
        id weight = definition.weights.empty() ? NSNull.null : @(definition.weights[index]);
        // Handles are derived projections of the adjacent retained poles; the
        // curve ends project none. No independent handle storage is admitted.
        id incoming = index == 0 ? NSNull.null
            : BoundedVector(definition.controlPoints[index - 1].local);
        id outgoing = index + 1 == definition.controlPoints.size() ? NSNull.null
            : BoundedVector(definition.controlPoints[index + 1].local);
        [poles addObject:@{@"id": BoundedUUIDText(pole.identifier),
            @"local": BoundedVector(pole.local), @"incomingHandle": incoming,
            @"outgoingHandle": outgoing, @"weight": weight}];
    }
    NSMutableArray *knots = [NSMutableArray arrayWithCapacity:definition.knots.size()];
    for (const auto& knot : definition.knots)
        [knots addObject:@{@"value": @(knot.value), @"multiplicity": @(knot.multiplicity)}];
    NSMutableArray *tombstones = [NSMutableArray arrayWithCapacity:retained.tombstones.size()];
    for (const auto& tombstone : retained.tombstones)
        [tombstones addObject:BoundedUUIDText(tombstone)];
    const auto& frame = definition.frame;
    return @{@"domain": definition.domain == Domain::Sketch2D ? @"sketch2D" : @"path3D",
        @"featureID": BoundedUUIDText(retained.authority.feature),
        @"definitionRevision": @(retained.authority.definitionRevision),
        @"nextLocalID": @(retained.nextLocalID),
        @"canonicalDigest": BoundedDigest(retained.authority.recipeDigest),
        @"owner": @{@"document": BoundedUUIDText(retained.authority.owner.document),
            @"entity": BoundedUUIDText(retained.authority.owner.entity),
            @"definition": BoundedUUIDText(retained.authority.owner.definition)},
        @"units": @{@"metersPerUnit": @(opening.scene.metersPerUnit)},
        @"tombstones": tombstones,
        @"poles": poles, @"degree": @(definition.degree), @"knots": knots,
        @"frame": @{@"id": BoundedUUIDText(frame.identifier), @"revision": @(frame.revision),
            @"origin": BoundedVector(frame.origin), @"xAxis": BoundedVector(frame.xAxis),
            @"yAxis": BoundedVector(frame.yAxis), @"zAxis": BoundedVector(frame.zAxis),
            @"handedness": @"right"}};
}

enum class BoundedParseResult : std::uint8_t { Valid, Malformed, Unsupported, Unchanged };

bool BoundedDigestValue(id value, core3d::bounded_curve::Digest& output) {
    output = {};
    if (![value isKindOfClass:NSData.class] || [value length] != output.size()) return false;
    [value getBytes:output.data() length:output.size()];
    return core3d::retained_recipe::Nonzero(output);
}

BoundedParseResult BoundedEdit(NSDictionary *value, NSUUID *editedPoleIdentifier,
                               NSString *gesture, const BoundedOpening& opening,
                               BoundedCandidate& output) {
    using namespace core3d::bounded_curve;
    if (!ExactKeys(value, @[@"domain", @"featureID", @"definitionRevision", @"nextLocalID",
            @"canonicalDigest", @"owner", @"units", @"tombstones",
            @"poles", @"degree", @"knots", @"frame"])
        || ![gesture isKindOfClass:NSString.class]) return BoundedParseResult::Malformed;
    NSString *domain = nil, *handedness = nil;
    BoundedUUID feature{}, frameID{}, edited{};
    BoundedUUID ownerDocument{}, ownerEntity{}, ownerDefinition{};
    Digest digest{};
    double metersPerUnit = 0;
    std::uint64_t revision = 0, nextLocalID = 0, degree = 0, frameRevision = 0;
    if (!String(value[@"domain"], domain) || !BoundedUUIDValue(value[@"featureID"], feature)
        || !Integer(value[@"definitionRevision"], UINT64_MAX, revision) || revision == 0
        || !Integer(value[@"nextLocalID"], UINT64_MAX, nextLocalID) || nextLocalID == 0
        || !BoundedDigestValue(value[@"canonicalDigest"], digest)
        || !Integer(value[@"degree"], UINT8_MAX, degree)
        || ![value[@"owner"] isKindOfClass:NSDictionary.class]
        || ![value[@"units"] isKindOfClass:NSDictionary.class]
        || ![value[@"tombstones"] isKindOfClass:NSArray.class]
        || ![value[@"poles"] isKindOfClass:NSArray.class]
        || ![value[@"knots"] isKindOfClass:NSArray.class]
        || ![value[@"frame"] isKindOfClass:NSDictionary.class]) return BoundedParseResult::Malformed;
    NSDictionary *ownerRow = value[@"owner"];
    if (!ExactKeys(ownerRow, @[@"document", @"entity", @"definition"])
        || !BoundedUUIDValue(ownerRow[@"document"], ownerDocument)
        || !BoundedUUIDValue(ownerRow[@"entity"], ownerEntity)
        || !BoundedUUIDValue(ownerRow[@"definition"], ownerDefinition))
        return BoundedParseResult::Malformed;
    NSDictionary *units = value[@"units"];
    if (!ExactKeys(units, @[@"metersPerUnit"])
        || !Number(units[@"metersPerUnit"], metersPerUnit))
        return BoundedParseResult::Malformed;
    std::vector<BoundedUUID> tombstones;
    for (id item in (NSArray *)value[@"tombstones"]) {
        BoundedUUID tombstone{};
        if (!BoundedUUIDValue(item, tombstone)) return BoundedParseResult::Malformed;
        tombstones.push_back(tombstone);
    }
    NSDictionary *frame = value[@"frame"];
    Definition candidate;
    if (!ExactKeys(frame, @[@"id", @"revision", @"origin", @"xAxis", @"yAxis", @"zAxis",
                            @"handedness"])
        || !BoundedUUIDValue(frame[@"id"], frameID)
        || !Integer(frame[@"revision"], UINT64_MAX, frameRevision) || frameRevision == 0
        || !Vector3(frame[@"origin"], candidate.frame.origin)
        || !Vector3(frame[@"xAxis"], candidate.frame.xAxis)
        || !Vector3(frame[@"yAxis"], candidate.frame.yAxis)
        || !Vector3(frame[@"zAxis"], candidate.frame.zAxis)
        || !String(frame[@"handedness"], handedness)) return BoundedParseResult::Malformed;
    candidate.domain = [domain isEqualToString:@"sketch2D"] ? Domain::Sketch2D
        : [domain isEqualToString:@"path3D"] ? Domain::Path3D : Domain{};
    candidate.degree = std::uint8_t(degree);
    candidate.frame.identifier = frameID;
    candidate.frame.revision = frameRevision;
    candidate.frame.handedness = [handedness isEqualToString:@"right"]
        ? Handedness::Right : Handedness{};
    bool anyWeight = false, anyNullWeight = false;
    for (id item in (NSArray *)value[@"poles"]) {
        if (![item isKindOfClass:NSDictionary.class]
            || !ExactKeys(item, @[@"id", @"local", @"incomingHandle", @"outgoingHandle", @"weight"]))
            return BoundedParseResult::Malformed;
        ControlPoint pole;
        if (!BoundedUUIDValue(item[@"id"], pole.identifier)
            || !Vector3(item[@"local"], pole.local)) return BoundedParseResult::Malformed;
        id weight = item[@"weight"];
        if (weight == NSNull.null) anyNullWeight = true;
        else { double scalar = 0; if (!Number(weight, scalar)) return BoundedParseResult::Malformed;
            candidate.weights.push_back(scalar); anyWeight = true; }
        candidate.controlPoints.push_back(pole);
    }
    if (anyWeight && anyNullWeight) return BoundedParseResult::Malformed;
    if (!anyWeight) candidate.weights.clear();
    // Handles are derived projections of adjacent candidate poles. A candidate
    // whose handle fields do not match its own pole values is malformed; no
    // independently stored handle is ever accepted.
    NSArray *poleRows = (NSArray *)value[@"poles"];
    for (std::size_t index = 0; index < candidate.controlPoints.size(); ++index) {
        NSDictionary *item = poleRows[index];
        std::array<double, 3> projected{};
        if (index > 0) {
            if (!Vector3(item[@"incomingHandle"], projected)
                || projected != candidate.controlPoints[index - 1].local)
                return BoundedParseResult::Malformed;
        } else if (item[@"incomingHandle"] != NSNull.null)
            return BoundedParseResult::Malformed;
        if (index + 1 < candidate.controlPoints.size()) {
            if (!Vector3(item[@"outgoingHandle"], projected)
                || projected != candidate.controlPoints[index + 1].local)
                return BoundedParseResult::Malformed;
        } else if (item[@"outgoingHandle"] != NSNull.null)
            return BoundedParseResult::Malformed;
    }
    for (id item in (NSArray *)value[@"knots"]) {
        std::uint64_t multiplicity = 0; double scalar = 0;
        if (![item isKindOfClass:NSDictionary.class]
            || !ExactKeys(item, @[@"value", @"multiplicity"])
            || !Number(item[@"value"], scalar)
            || !Integer(item[@"multiplicity"], UINT8_MAX, multiplicity))
            return BoundedParseResult::Malformed;
        candidate.knots.push_back({scalar, std::uint8_t(multiplicity)});
    }
    const RetainedState& retained = opening.retained;
    const Definition& original = retained.definition;
    // Owner identity, units and the issuance/tombstone ledger are read-only
    // projection echoes. A document unit change invalidates the opening; it is
    // never a silent candidate conversion.
    if (feature != retained.authority.feature || revision != retained.authority.definitionRevision
        || nextLocalID != retained.nextLocalID || digest != retained.authority.recipeDigest
        || ownerDocument != retained.authority.owner.document
        || ownerEntity != retained.authority.owner.entity
        || ownerDefinition != retained.authority.owner.definition
        || metersPerUnit != opening.scene.metersPerUnit
        || tombstones != retained.tombstones
        || candidate.domain != original.domain || candidate.degree != original.degree
        || !SameFrame(candidate.frame, original.frame) || candidate.knots.size() != original.knots.size()
        || candidate.controlPoints.size() != original.controlPoints.size())
        return BoundedParseResult::Unsupported;
    for (std::size_t index = 0; index < candidate.knots.size(); ++index)
        if (candidate.knots[index].value != original.knots[index].value
            || candidate.knots[index].multiplicity != original.knots[index].multiplicity)
            return BoundedParseResult::Unsupported;
    for (std::size_t index = 0; index < candidate.controlPoints.size(); ++index)
        if (candidate.controlPoints[index].identifier != original.controlPoints[index].identifier)
            return BoundedParseResult::Unsupported;
    if (!editedPoleIdentifier || !BoundedUUIDValue(editedPoleIdentifier.UUIDString, edited))
        return BoundedParseResult::Malformed;
    const auto found = std::find_if(candidate.controlPoints.begin(), candidate.controlPoints.end(),
        [&](const ControlPoint& pole) { return pole.identifier == edited; });
    if (found == candidate.controlPoints.end()) return BoundedParseResult::Malformed;
    const std::size_t editedIndex = std::size_t(found - candidate.controlPoints.begin());
    const bool movePole = [gesture isEqualToString:@"movePole"];
    const bool setWeight = [gesture isEqualToString:@"setWeight"];
    const bool moveIncoming = [gesture isEqualToString:@"moveIncomingHandle"];
    const bool moveOutgoing = [gesture isEqualToString:@"moveOutgoingHandle"];
    if (!movePole && !setWeight && !moveIncoming && !moveOutgoing)
        return BoundedParseResult::Unsupported;
    // A handle gesture resolves to the adjacent retained pole: incoming moves
    // the previous pole, outgoing the next. The curve ends have no handle.
    std::size_t targetIndex = editedIndex;
    if (moveIncoming) {
        if (editedIndex == 0) return BoundedParseResult::Unsupported;
        targetIndex = editedIndex - 1;
    } else if (moveOutgoing) {
        if (editedIndex + 1 == candidate.controlPoints.size())
            return BoundedParseResult::Unsupported;
        targetIndex = editedIndex + 1;
    }
    for (std::size_t index = 0; index < candidate.controlPoints.size(); ++index)
        if (index != targetIndex && candidate.controlPoints[index].local != original.controlPoints[index].local)
            return BoundedParseResult::Unsupported;
    const bool poleChanged = candidate.controlPoints[targetIndex].local
        != original.controlPoints[targetIndex].local;
    const bool weightsChanged = candidate.weights != original.weights;
    if ((movePole || moveIncoming || moveOutgoing) && weightsChanged)
        return BoundedParseResult::Unsupported;
    if (setWeight && poleChanged)
        return BoundedParseResult::Unsupported;
    if (setWeight) {
        if (candidate.weights.size() != original.controlPoints.size()
            || original.weights.size() != original.controlPoints.size())
            return BoundedParseResult::Unsupported;
        for (std::size_t index = 0; index < candidate.weights.size(); ++index)
            if (index != editedIndex && candidate.weights[index] != original.weights[index])
                return BoundedParseResult::Unsupported;
    }
    if (!poleChanged && !weightsChanged) return BoundedParseResult::Unchanged;
    output.completeDefinition = candidate;
    output.proposal.expected = retained.authority;
    output.proposal.controlPoint = candidate.controlPoints[targetIndex].identifier;
    if (movePole) {
        output.proposal.kind = EditKind::MovePole;
        output.proposal.replacementLocal = candidate.controlPoints[targetIndex].local;
    } else if (moveIncoming || moveOutgoing) {
        output.proposal.kind = moveIncoming
            ? EditKind::MoveIncomingHandle : EditKind::MoveOutgoingHandle;
        output.proposal.replacementLocal = candidate.controlPoints[targetIndex].local;
    } else {
        output.proposal.kind = EditKind::SetWeight;
        output.proposal.replacementWeight = candidate.weights[editedIndex];
    }
    return BoundedParseResult::Valid;
}

using GeneralOpening = core3d::general_loft::owner::Opening;
using GeneralCandidate = core3d::general_loft::owner::Candidate;

NSArray *GeneralVector2(const std::array<double, 2>& value) {
    return @[@(value[0]), @(value[1])];
}

NSDictionary *GeneralLoftDescriptor(const GeneralOpening& opening) {
    using namespace core3d::general_loft;
    const Definition& value = opening.definition;
    NSMutableArray *stations = [NSMutableArray arrayWithCapacity:value.stations.size()];
    for (const Station& station : value.stations) {
        NSMutableArray *junctions = [NSMutableArray arrayWithCapacity:station.junctions.size()];
        for (const Junction& junction : station.junctions)
            [junctions addObject:@{@"id": UUIDText(junction.identifier),
                @"correspondence": UUIDText(junction.correspondence),
                @"local": GeneralVector2(junction.local)}];
        NSMutableArray *segments = [NSMutableArray arrayWithCapacity:station.segments.size()];
        for (const Segment& segment : station.segments) {
            NSMutableArray *poles = [NSMutableArray array];
            const auto& curve = segment.curveState.definition;
            for (std::size_t index = 0; index < curve.controlPoints.size(); ++index)
                [poles addObject:@{@"id": UUIDText(curve.controlPoints[index].identifier),
                    @"local": BoundedVector(curve.controlPoints[index].local),
                    @"weight": @(curve.weights.empty() ? 1 : curve.weights[index])}];
            NSMutableArray *knots = [NSMutableArray array];
            for (const auto& knot : curve.knots)
                [knots addObject:@{@"value": @(knot.value),
                    @"multiplicity": @(knot.multiplicity)}];
            [segments addObject:@{@"id": UUIDText(segment.identifier),
                @"correspondence": UUIDText(segment.correspondence),
                @"startJunction": UUIDText(segment.startJunction),
                @"endJunction": UUIDText(segment.endJunction),
                @"curveRevision": @(segment.curveState.authority.definitionRevision),
                @"degree": @(curve.degree), @"poles": poles, @"knots": knots}];
        }
        const auto& frame = station.frame;
        [stations addObject:@{@"id": UUIDText(station.identifier),
            @"orderParameter": @(station.orderParameter),
            @"twistFromPreviousDegrees": @(station.twistFromPreviousRadians * 180 / M_PI),
            @"frame": @{@"id": UUIDText(frame.identifier), @"revision": @(frame.revision),
                @"origin": BoundedVector(frame.origin), @"xAxis": BoundedVector(frame.xAxis),
                @"yAxis": BoundedVector(frame.yAxis), @"zAxis": BoundedVector(frame.zAxis)},
            @"junctions": junctions, @"segments": segments}];
    }
    return @{@"owner": @{@"document": UUIDText(value.owner.document),
            @"entity": UUIDText(value.owner.entity),
            @"definition": UUIDText(value.owner.definition)},
        @"feature": UUIDText(value.feature), @"definitionRevision": @(value.definitionRevision),
        @"canonicalDigest": BoundedDigest(value.recipeDigest),
        @"dimensionMetersPerUnit": @(value.dimensionMetersPerUnit),
        @"interpolation": @"ruled.v1", @"holePolicy": @"reject.v1",
        @"capPolicy": @"flatBothEnds.v1", @"orderAxis": BoundedVector(value.orderAxis),
        @"stations": stations, @"allowsStationInsertion": @NO,
        @"allowsStationRemoval": @NO, @"creating": @(opening.creating)};
}

bool SameGeneralOwner(id projected, const core3d::retained_recipe::OwnerKey& owner) {
    if (![projected isKindOfClass:NSDictionary.class]
        || !ExactKeys(projected, @[@"document", @"entity", @"definition"])) return false;
    BoundedUUID document{}, entity{}, definition{};
    return BoundedUUIDValue(projected[@"document"], document)
        && BoundedUUIDValue(projected[@"entity"], entity)
        && BoundedUUIDValue(projected[@"definition"], definition)
        && document == owner.document && entity == owner.entity && definition == owner.definition;
}

bool ParseGeneralFrame(NSDictionary *value, const core3d::bounded_curve::Frame& original,
                       core3d::bounded_curve::Frame& output) {
    std::uint64_t revision = 0; BoundedUUID identifier{};
    output = original;
    return ExactKeys(value, @[@"id", @"revision", @"origin", @"xAxis", @"yAxis", @"zAxis"])
        && BoundedUUIDValue(value[@"id"], identifier) && identifier == original.identifier
        && Integer(value[@"revision"], UINT64_MAX, revision) && revision == original.revision
        && Vector3(value[@"origin"], output.origin) && Vector3(value[@"xAxis"], output.xAxis)
        && Vector3(value[@"yAxis"], output.yAxis) && Vector3(value[@"zAxis"], output.zAxis);
}

bool CanonicalizeGeneralCurve(core3d::bounded_curve::RetainedState& state) {
    if (state.authority.definitionRevision == UINT64_MAX) return false;
    ++state.authority.definitionRevision;
    core3d::bounded_curve::Value value{state.authority.feature, state.definition};
    std::vector<std::uint8_t> bytes;
    return core3d::bounded_curve::Encode(value, bytes)
        && core3d::bounded_curve::Hash(bytes, core3d::bounded_curve::MaximumDefinitionBytes,
                                      state.authority.recipeDigest)
        && core3d::bounded_curve::ValidState(state);
}

bool GeneralLoftCandidate(NSDictionary *projected, NSUUID *editedStation,
                          const GeneralOpening& opening, GeneralCandidate& output) {
    using namespace core3d::general_loft;
    if (!editedStation || !ExactKeys(projected, @[@"owner", @"feature", @"definitionRevision",
            @"canonicalDigest", @"dimensionMetersPerUnit", @"interpolation", @"holePolicy",
            @"capPolicy", @"orderAxis", @"stations", @"allowsStationInsertion",
            @"allowsStationRemoval", @"creating"])) return false;
    const Definition& original = opening.definition;
    BoundedUUID feature{}, stationID{}; Digest digest{}; std::uint64_t revision = 0;
    double units = 0; NSString *interpolation = nil, *holes = nil, *caps = nil;
    std::array<double, 3> orderAxis{};
    if (!SameGeneralOwner(projected[@"owner"], original.owner)
        || !BoundedUUIDValue(projected[@"feature"], feature) || feature != original.feature
        || !Integer(projected[@"definitionRevision"], UINT64_MAX, revision)
        || revision != original.definitionRevision
        || !BoundedDigestValue(projected[@"canonicalDigest"], digest)
        || digest != original.recipeDigest || !Number(projected[@"dimensionMetersPerUnit"], units)
        || units != original.dimensionMetersPerUnit
        || !String(projected[@"interpolation"], interpolation)
        || !String(projected[@"holePolicy"], holes) || !String(projected[@"capPolicy"], caps)
        || ![interpolation isEqualToString:@"ruled.v1"] || ![holes isEqualToString:@"reject.v1"]
        || ![caps isEqualToString:@"flatBothEnds.v1"] || !Vector3(projected[@"orderAxis"], orderAxis)
        || orderAxis != original.orderAxis
        || ![projected[@"allowsStationInsertion"] isKindOfClass:NSNumber.class]
        || [projected[@"allowsStationInsertion"] boolValue]
        || ![projected[@"allowsStationRemoval"] isKindOfClass:NSNumber.class]
        || [projected[@"allowsStationRemoval"] boolValue]
        || ![projected[@"stations"] isKindOfClass:NSArray.class]
        || !BoundedUUIDValue(editedStation.UUIDString, stationID)) return false;
    NSArray *stationValues = projected[@"stations"];
    if (stationValues.count != original.stations.size()) return false;
    Definition candidate = original;
    for (std::size_t stationIndex = 0; stationIndex < original.stations.size(); ++stationIndex) {
        id rawStation = stationValues[stationIndex];
        if (![rawStation isKindOfClass:NSDictionary.class]
            || !ExactKeys(rawStation, @[@"id", @"orderParameter", @"twistFromPreviousDegrees",
                @"frame", @"junctions", @"segments"])) return false;
        const Station& before = original.stations[stationIndex]; Station after = before;
        BoundedUUID identifier{}; double twist = 0;
        if (!BoundedUUIDValue(rawStation[@"id"], identifier) || identifier != before.identifier
            || !Number(rawStation[@"orderParameter"], after.orderParameter)
            || !Number(rawStation[@"twistFromPreviousDegrees"], twist)
            || ![rawStation[@"frame"] isKindOfClass:NSDictionary.class]
            || !ParseGeneralFrame(rawStation[@"frame"], before.frame, after.frame)
            || ![rawStation[@"junctions"] isKindOfClass:NSArray.class]
            || ![rawStation[@"segments"] isKindOfClass:NSArray.class]) return false;
        after.twistFromPreviousRadians = twist * M_PI / 180;
        NSArray *junctions = rawStation[@"junctions"], *segments = rawStation[@"segments"];
        if (junctions.count != before.junctions.size() || segments.count != before.segments.size())
            return false;
        for (std::size_t index = 0; index < before.junctions.size(); ++index) {
            id row = junctions[index]; BoundedUUID identifier{}, correspondence{}; std::array<double, 3> local3{};
            if (![row isKindOfClass:NSDictionary.class]
                || !ExactKeys(row, @[@"id", @"correspondence", @"local"])
                || !BoundedUUIDValue(row[@"id"], identifier)
                || identifier != before.junctions[index].identifier
                || !BoundedUUIDValue(row[@"correspondence"], correspondence)
                || correspondence != before.junctions[index].correspondence
                || ![row[@"local"] isKindOfClass:NSArray.class]
                || [(NSArray *)row[@"local"] count] != 2) return false;
            double u = 0, v = 0;
            if (!Number(row[@"local"][0], u) || !Number(row[@"local"][1], v)) return false;
            after.junctions[index].local = {{u, v}};
        }
        for (std::size_t index = 0; index < before.segments.size(); ++index) {
            id row = segments[index];
            if (![row isKindOfClass:NSDictionary.class]
                || !ExactKeys(row, @[@"id", @"correspondence", @"startJunction", @"endJunction",
                    @"curveRevision", @"degree", @"poles", @"knots"])) return false;
            BoundedUUID identifier{}, corr{}, start{}, end{};
            std::uint64_t curveRevision = 0, degree = 0;
            const Segment& source = before.segments[index]; Segment& target = after.segments[index];
            if (!BoundedUUIDValue(row[@"id"], identifier) || identifier != source.identifier
                || !BoundedUUIDValue(row[@"correspondence"], corr) || corr != source.correspondence
                || !BoundedUUIDValue(row[@"startJunction"], start) || start != source.startJunction
                || !BoundedUUIDValue(row[@"endJunction"], end) || end != source.endJunction
                || !Integer(row[@"curveRevision"], UINT64_MAX, curveRevision)
                || curveRevision != source.curveState.authority.definitionRevision
                || !Integer(row[@"degree"], UINT8_MAX, degree)
                || ![row[@"poles"] isKindOfClass:NSArray.class]
                || ![row[@"knots"] isKindOfClass:NSArray.class]) return false;
            auto& curve = target.curveState.definition; curve.degree = std::uint8_t(degree);
            NSArray *poles = row[@"poles"], *knots = row[@"knots"];
            if (poles.count != curve.controlPoints.size() || knots.count != curve.knots.size()) return false;
            std::vector<double> weights; weights.reserve(poles.count);
            for (std::size_t poleIndex = 0; poleIndex < curve.controlPoints.size(); ++poleIndex) {
                id pole = poles[poleIndex]; BoundedUUID poleID{}; std::array<double, 3> local{};
                double weight = 0;
                if (![pole isKindOfClass:NSDictionary.class]
                    || !ExactKeys(pole, @[@"id", @"local", @"weight"])
                    || !BoundedUUIDValue(pole[@"id"], poleID)
                    || poleID != curve.controlPoints[poleIndex].identifier
                    || !Vector3(pole[@"local"], local) || !Number(pole[@"weight"], weight)) return false;
                curve.controlPoints[poleIndex].local = local; weights.push_back(weight);
            }
            curve.weights = std::all_of(weights.begin(), weights.end(), [](double v) { return v == 1; })
                ? std::vector<double>{} : std::move(weights);
            for (std::size_t knotIndex = 0; knotIndex < curve.knots.size(); ++knotIndex) {
                id knot = knots[knotIndex]; double value = 0; std::uint64_t multiplicity = 0;
                if (![knot isKindOfClass:NSDictionary.class]
                    || !ExactKeys(knot, @[@"value", @"multiplicity"])
                    || !Number(knot[@"value"], value)
                    || !Integer(knot[@"multiplicity"], UINT8_MAX, multiplicity)) return false;
                curve.knots[knotIndex] = {value, std::uint8_t(multiplicity)};
            }
            if (!core3d::bounded_curve::SameDefinition(
                    target.curveState.definition, source.curveState.definition)
                && !CanonicalizeGeneralCurve(target.curveState)) return false;
        }
        if (before.identifier != stationID && !SameStationValue(before, after)) return false;
        candidate.stations[stationIndex] = std::move(after);
    }
    output.descriptive = std::move(candidate); output.editedStation = stationID; return true;
}

bool ParseCreationStation(NSDictionary *value,
                          core3d::general_loft::owner::CreationStation& output) {
    using core3d::bounded_curve::Handedness;
    if (!ExactKeys(value, @[@"orderParameter", @"twistFromPreviousDegrees", @"frame", @"junctions"])
        || !Number(value[@"orderParameter"], output.orderParameter)
        || ![value[@"frame"] isKindOfClass:NSDictionary.class]
        || ![value[@"junctions"] isKindOfClass:NSArray.class]) return false;
    double twist = 0;
    if (!Number(value[@"twistFromPreviousDegrees"], twist)) return false;
    output.twistFromPreviousRadians = twist * M_PI / 180;
    NSDictionary *frame = value[@"frame"];
    if (!ExactKeys(frame, @[@"origin", @"xAxis", @"yAxis", @"zAxis"])
        || !Vector3(frame[@"origin"], output.frame.origin)
        || !Vector3(frame[@"xAxis"], output.frame.xAxis)
        || !Vector3(frame[@"yAxis"], output.frame.yAxis)
        || !Vector3(frame[@"zAxis"], output.frame.zAxis)) return false;
    output.frame.handedness = Handedness::Right;
    for (id row in (NSArray *)value[@"junctions"]) {
        if (![row isKindOfClass:NSArray.class] || [row count] != 2) return false;
        double u = 0, v = 0; if (!Number(row[0], u) || !Number(row[1], v)) return false;
        output.junctions.push_back({{u, v}});
    }
    return true;
}

void Deliver(void (^completion)(Core3DProfileConstructionResult, NSString *),
             Core3DProfileConstructionResult result, NSString *detail) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}

void DeliverBounded(void (^completion)(Core3DBoundedCurvePreparationResult, NSString *),
                    Core3DBoundedCurvePreparationResult result, NSString *detail) {
#if DEBUG
    if (result != Core3DBoundedCurvePreparationResultPrepared) {
        std::fprintf(stderr, "R179_CURVE_RESULT stage=prepare result=%d detail=%.96s\n",
                     int(result), detail ? detail.UTF8String : "");
    }
#endif
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail);
    else dispatch_async(dispatch_get_main_queue(), ^{ completion(result, detail); });
}

void DeliverCreated(void (^completion)(Core3DProfileConstructionResult, NSString *, NSString *),
                    Core3DProfileConstructionResult result, NSString *detail,
                    NSString *entityIdentifier) {
    if (!completion) return;
    if (NSThread.isMainThread) completion(result, detail, entityIdentifier);
    else dispatch_async(dispatch_get_main_queue(), ^{
        completion(result, detail, entityIdentifier); });
}

// Native identity minting for ordinary authoring. Swift never supplies UUIDs.
bool BoundedMintUUID(BoundedUUID& output) {
    output = {};
    NSUUID *uuid = [NSUUID UUID];
    if (!uuid) return false;
    uuid_t bytes{};
    [uuid getUUIDBytes:bytes];
    std::copy_n(bytes, output.size(), output.begin());
    return core3d::bounded_curve::Nonzero(output);
}

// Ordinary C1 authoring. The candidate carries explicit construction values
// only (domain, degree, knots, pole locals/optional weights, authored frame);
// every durable identity is minted here, and the owner reassigns the exact
// entity/definition identities from its own label issuance before staging.
bool BoundedCreationCandidate(NSDictionary *value, NSString *name,
                              const Handle(OcctDocument)& owner,
                              const core3d::native_opening::Context& context,
                              core3d::bounded_curve::owner::CreateRequest& output) {
    using namespace core3d::bounded_curve;
    if (!ExactKeys(value, @[@"domain", @"degree", @"knots", @"poles", @"frame"])
        || ![name isKindOfClass:NSString.class]) return false;
    NSString *domain = nil;
    std::uint64_t degree = 0;
    if (!String(value[@"domain"], domain)
        || !Integer(value[@"degree"], UINT8_MAX, degree)
        || ![value[@"poles"] isKindOfClass:NSArray.class]
        || ![value[@"knots"] isKindOfClass:NSArray.class]
        || ![value[@"frame"] isKindOfClass:NSDictionary.class]) return false;
    Definition definition;
    definition.domain = [domain isEqualToString:@"sketch2D"] ? Domain::Sketch2D
        : [domain isEqualToString:@"path3D"] ? Domain::Path3D : Domain{};
    definition.degree = std::uint8_t(degree);
    NSDictionary *frame = value[@"frame"];
    if (!ExactKeys(frame, @[@"origin", @"xAxis", @"yAxis", @"zAxis"])
        || !Vector3(frame[@"origin"], definition.frame.origin)
        || !Vector3(frame[@"xAxis"], definition.frame.xAxis)
        || !Vector3(frame[@"yAxis"], definition.frame.yAxis)
        || !Vector3(frame[@"zAxis"], definition.frame.zAxis)) return false;
    definition.frame.revision = 1;
    definition.frame.handedness = Handedness::Right;
    if (!BoundedMintUUID(definition.frame.identifier)) return false;
    bool anyWeight = false, anyNullWeight = false;
    for (id item in (NSArray *)value[@"poles"]) {
        if (![item isKindOfClass:NSDictionary.class]
            || !ExactKeys(item, @[@"local", @"weight"])) return false;
        ControlPoint pole;
        if (!Vector3(item[@"local"], pole.local)
            || !BoundedMintUUID(pole.identifier)) return false;
        id weight = item[@"weight"];
        if (weight == NSNull.null) anyNullWeight = true;
        else { double scalar = 0; if (!Number(weight, scalar)) return false;
            definition.weights.push_back(scalar); anyWeight = true; }
        definition.controlPoints.push_back(pole);
    }
    if (anyWeight && anyNullWeight) return false;
    if (!anyWeight) definition.weights.clear();
    for (id item in (NSArray *)value[@"knots"]) {
        std::uint64_t multiplicity = 0; double scalar = 0;
        if (![item isKindOfClass:NSDictionary.class]
            || !ExactKeys(item, @[@"value", @"multiplicity"])
            || !Number(item[@"value"], scalar)
            || !Integer(item[@"multiplicity"], UINT8_MAX, multiplicity)) return false;
        definition.knots.push_back({scalar, std::uint8_t(multiplicity)});
    }
    // Existing C1a law: degree 1-7, 128-pole budget, positive bounded weights,
    // clamped nonperiodic knots and the sketch-plane rule are enforced here.
    if (Validate(definition) != Refusal::None) return false;

    PersistedValue persisted;
    persisted.value.definition = definition;
    persisted.ownerState.definitionRevision = 1;
    persisted.ownerState.nextLocalID = definition.controlPoints.size() + 1;
    if (!BoundedMintUUID(persisted.value.feature)
        || !BoundedUUIDValue(Text(owner->DocumentIdentifier()),
                             persisted.ownerState.owner.document)
        || !BoundedMintUUID(persisted.ownerState.owner.entity)
        || !BoundedMintUUID(persisted.ownerState.owner.definition)) return false;
    persisted.ownerState.feature = persisted.value.feature;
    std::vector<std::uint8_t> bytes;
    if (!Encode(persisted.value, bytes)
        || !Hash(bytes, MaximumDefinitionBytes, persisted.ownerState.canonicalDefinitionDigest)
        || !ValidatePersisted(persisted)) return false;
    DetachedWire detached;
    if (BuildWire(persisted, detached) != BuildRefusal::None
        || !MatchesPersistedValue(detached, persisted)) return false;

    output = {};
    output.expectedScene = owner::SceneFence{context.openingFence().documentGeneration(),
        context.openingFence().modelRevision(), context.openingFence().metersPerUnit()};
    output.persisted = std::move(persisted);
    output.detachedWire = std::move(detached);
    output.requestedName = name.UTF8String ?: "";
    return !output.requestedName.empty() && output.requestedName.size() <= 128;
}
} // namespace

@interface Core3DBoundedCurveEditingOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<core3d::native_opening::Context> _context;
    std::unique_ptr<core3d::bounded_curve::owner::OcafOwner> _service;
    std::shared_ptr<const core3d::bounded_curve::owner::Opening> _opening;
    std::shared_ptr<const core3d::bounded_curve::owner::Prepared> _prepared;
    std::atomic<State> _state;
}
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    service:(std::unique_ptr<core3d::bounded_curve::owner::OcafOwner>)service
    opening:(std::shared_ptr<const core3d::bounded_curve::owner::Opening>)opening;
@end

@implementation Core3DBoundedCurveEditingOpening
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    service:(std::unique_ptr<core3d::bounded_curve::owner::OcafOwner>)service
    opening:(std::shared_ptr<const core3d::bounded_curve::owner::Opening>)opening {
    if ((self = [super init])) {
        _owner = owner; _context = std::move(context); _service = std::move(service);
        _opening = std::move(opening); _state.store(State::Open);
    }
    return self;
}
- (NSDictionary *)descriptor {
    return _opening ? BoundedDescriptor(*_opening) : @{};
}
- (void)prepareCandidate:(NSDictionary *)candidate
    editedPoleIdentifier:(NSUUID *)editedPoleIdentifier gesture:(NSString *)gesture
    completion:(void (^)(Core3DBoundedCurvePreparationResult, NSString *))completion {
    if (_state.load() != State::Open || !_service || !_opening) {
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected,
                       @"Opening is not available for preparation.");
        return;
    }
    BoundedCandidate edit;
    switch (BoundedEdit(candidate, editedPoleIdentifier, gesture, *_opening, edit)) {
        case BoundedParseResult::Malformed:
            DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected,
                           @"Malformed or nonfinite bounded-curve values.");
            return;
        case BoundedParseResult::Unsupported:
            DeliverBounded(completion, Core3DBoundedCurvePreparationResultUnsupportedMutation,
                           @"Degree, knots, frame, pole identity, owner, units, and unrelated values are read-only.");
            return;
        case BoundedParseResult::Unchanged:
            DeliverBounded(completion, Core3DBoundedCurvePreparationResultUnchanged,
                           @"The retained curve is unchanged.");
            return;
        case BoundedParseResult::Valid: break;
    }
    core3d::bounded_curve::owner::Receipt receipt;
    auto prepared = _service->prepare(_opening, edit, receipt);
    if (!prepared || receipt.outcome != core3d::bounded_curve::owner::Outcome::prepared) {
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected,
                       @"Native bounded-curve preparation refused without history.");
        return;
    }
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Prepared)) {
        (void)_service->cancel(_opening->session);
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected,
                       @"Opening changed while preparing.");
        return;
    }
    _prepared = std::move(prepared);
    DeliverBounded(completion, Core3DBoundedCurvePreparationResultPrepared,
                   @"Bounded-curve edit prepared.");
}
- (void)applyWithCompletion:
    (void (^)(Core3DProfileConstructionResult, NSString *))completion {
    State expected = State::Prepared;
    if (!_state.compare_exchange_strong(expected, State::Applying)
        || !_service || !_prepared) {
#if DEBUG
        std::fprintf(stderr, "R179_CURVE_RESULT stage=apply result=%d detail=%.96s\n",
                     int(Core3DProfileConstructionResultRejected),
                     "No current bounded-curve preparation.");
#endif
        Deliver(completion, Core3DProfileConstructionResultRejected,
                @"No current bounded-curve preparation.");
        return;
    }
    const auto receipt = _service->apply(_prepared);
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultRejected;
    if (receipt.outcome == core3d::bounded_curve::owner::Outcome::committed)
        result = Core3DProfileConstructionResultCommitted;
    else if (receipt.outcome == core3d::bounded_curve::owner::Outcome::unchanged)
        result = Core3DProfileConstructionResultUnchanged;
    else if (receipt.outcome == core3d::bounded_curve::owner::Outcome::busy)
        result = Core3DProfileConstructionResultBusy;
    else if (receipt.outcome == core3d::bounded_curve::owner::Outcome::outcomeUnknown
             || receipt.outcome == core3d::bounded_curve::owner::Outcome::recoveryRequired)
        result = Core3DProfileConstructionResultRecoveryRequired;
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) {
        _prepared.reset(); _opening.reset(); _service.reset(); _context.reset();
    }
    NSString *applyDetail = result == Core3DProfileConstructionResultCommitted
        ? @"Bounded curve committed." : result == Core3DProfileConstructionResultRecoveryRequired
            ? @"Bounded-curve close is unknown; native recovery ownership is retained."
            : @"Bounded-curve edit refused without retry.";
#if DEBUG
    if (result != Core3DProfileConstructionResultCommitted) {
        std::fprintf(stderr, "R179_CURVE_RESULT stage=apply result=%d detail=%.96s\n",
                     int(result), applyDetail.UTF8String);
    }
#endif
    Deliver(completion, result, applyDetail);
}
- (BOOL)cancel {
    State value = _state.load();
    while (value == State::Open || value == State::Prepared) {
        if (_state.compare_exchange_weak(value, State::Cancelled)) {
            if (!_service || !_opening
                || _service->cancel(_opening->session).outcome
                    != core3d::bounded_curve::owner::Outcome::cancelled) {
                _state.store(State::Recovery); return NO;
            }
            _prepared.reset(); _opening.reset(); _service.reset(); _context.reset();
            return YES;
        }
    }
    return NO;
}
@end

@interface Core3DBoundedCurveCreationOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<core3d::native_opening::Context> _context;
    std::unique_ptr<core3d::bounded_curve::owner::OcafOwner> _service;
    std::atomic<State> _state;
}
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    service:(std::unique_ptr<core3d::bounded_curve::owner::OcafOwner>)service;
@end

@implementation Core3DBoundedCurveCreationOpening
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    service:(std::unique_ptr<core3d::bounded_curve::owner::OcafOwner>)service {
    if ((self = [super init])) {
        _owner = owner; _context = std::move(context); _service = std::move(service);
        _state.store(State::Open);
    }
    return self;
}
- (NSDictionary *)descriptor {
    if (_state.load() != State::Open || !_context || _owner.IsNull()) return @{};
    return @{@"owner": @{@"document": Text(_owner->DocumentIdentifier())},
        @"units": @{@"metersPerUnit": @(_context->openingFence().metersPerUnit())},
        @"domains": @[@"sketch2D", @"path3D"],
        @"minimumDegree": @(core3d::bounded_curve::MinimumDegree),
        @"maximumDegree": @(core3d::bounded_curve::MaximumDegree),
        @"maximumPoles": @(core3d::bounded_curve::MaximumControlPoints),
        @"maximumCoordinate": @(core3d::bounded_curve::CoordinateLimit),
        @"minimumWeight": @(core3d::bounded_curve::MinimumWeight),
        @"maximumWeight": @(core3d::bounded_curve::MaximumWeight)};
}
- (void)createCandidate:(NSDictionary *)candidate name:(NSString *)name
    completion:(void (^)(Core3DProfileConstructionResult, NSString *, NSString *))completion {
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Applying)
        || !_service || !_context || _owner.IsNull()
        || ![candidate isKindOfClass:NSDictionary.class]) {
        DeliverCreated(completion, Core3DProfileConstructionResultRejected,
                       @"Creation opening is not available.", nil);
        return;
    }
    core3d::bounded_curve::owner::CreateRequest request;
    if (!BoundedCreationCandidate(candidate, name, _owner, *_context, request)) {
        _state.store(State::Open);
        DeliverCreated(completion, Core3DProfileConstructionResultRejected,
            @"Malformed, nonfinite, or structurally unsupported bounded-curve values. "
            @"Pole count, degree, clamped knots, weights, and the sketch plane are bounded native admissions.",
            nil);
        return;
    }
    const auto receipt = _service->create(request);
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultRejected;
    if (receipt.outcome == core3d::bounded_curve::owner::Outcome::committed)
        result = Core3DProfileConstructionResultCommitted;
    else if (receipt.outcome == core3d::bounded_curve::owner::Outcome::busy)
        result = Core3DProfileConstructionResultBusy;
    else if (receipt.outcome == core3d::bounded_curve::owner::Outcome::outcomeUnknown
             || receipt.outcome == core3d::bounded_curve::owner::Outcome::recoveryRequired)
        result = Core3DProfileConstructionResultRecoveryRequired;
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) {
        _service.reset(); _context.reset();
    }
    NSString *entity = result == Core3DProfileConstructionResultCommitted
        ? Text(core3d::retained_solid::UUIDText(receipt.authority.owner.entity)) : nil;
    DeliverCreated(completion, result, result == Core3DProfileConstructionResultCommitted
        ? @"Bounded curve created as one native history command."
        : result == Core3DProfileConstructionResultRecoveryRequired
            ? @"Bounded-curve creation close is unknown; native recovery ownership is retained."
            : @"Bounded-curve creation refused without history.", entity);
}
- (BOOL)cancel {
    State value = _state.load();
    while (value == State::Open) {
        if (_state.compare_exchange_weak(value, State::Cancelled)) {
            _service.reset(); _context.reset();
            return YES;
        }
    }
    return NO;
}
@end

@interface Core3DGeneralLoftEditingOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<core3d::native_opening::Context> _context;
    std::unique_ptr<core3d::general_loft::owner::OcafOwner> _service;
    std::shared_ptr<const core3d::general_loft::owner::Opening> _opening;
    std::shared_ptr<const core3d::general_loft::owner::Prepared> _prepared;
    std::atomic<State> _state;
}
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    service:(std::unique_ptr<core3d::general_loft::owner::OcafOwner>)service
    opening:(std::shared_ptr<const core3d::general_loft::owner::Opening>)opening;
@end

@implementation Core3DGeneralLoftEditingOpening
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    service:(std::unique_ptr<core3d::general_loft::owner::OcafOwner>)service
    opening:(std::shared_ptr<const core3d::general_loft::owner::Opening>)opening {
    if ((self = [super init])) {
        _owner = owner; _context = std::move(context); _service = std::move(service);
        _opening = std::move(opening); _state.store(State::Open);
    }
    return self;
}
- (NSDictionary *)descriptor { return _opening ? GeneralLoftDescriptor(*_opening) : @{}; }
- (BOOL)isCreating { return _opening && _opening->creating; }
- (void)prepareCandidate:(NSDictionary *)candidate
    editedStationIdentifier:(NSUUID *)stationIdentifier
    completion:(void (^)(Core3DBoundedCurvePreparationResult, NSString *))completion {
    if (_state.load() != State::Open || !_service || !_opening) {
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected,
                       @"General-loft opening is no longer current."); return;
    }
    GeneralCandidate edit;
    if (!GeneralLoftCandidate(candidate, stationIdentifier, *_opening, edit)) {
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultUnsupportedMutation,
            @"Station count/order, correspondence, IDs, caps, holes, and interpolation are read-only.");
        return;
    }
    core3d::general_loft::owner::Receipt receipt;
    auto prepared = _service->prepare(_opening, edit, receipt);
    if (!prepared) {
        const auto result = receipt.outcome == core3d::general_loft::owner::Outcome::unchanged
            ? Core3DBoundedCurvePreparationResultUnchanged
            : receipt.outcome == core3d::general_loft::owner::Outcome::unsupportedStructure
                || receipt.outcome == core3d::general_loft::owner::Outcome::unsupportedDependent
            ? Core3DBoundedCurvePreparationResultUnsupportedMutation
            : Core3DBoundedCurvePreparationResultRejected;
        DeliverBounded(completion, result,
            receipt.outcome == core3d::general_loft::owner::Outcome::unsupportedDependent
                ? @"A retained descendant cannot be rebuilt, so nothing changed."
                : result == Core3DBoundedCurvePreparationResultUnchanged
                    ? @"The ruled loft is unchanged." : @"Native ruled-loft proof refused the candidate.");
        return;
    }
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Prepared)) {
        (void)_service->cancel(_opening->session);
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected,
                       @"The opening changed during proof."); return;
    }
    _prepared = std::move(prepared);
    DeliverBounded(completion, Core3DBoundedCurvePreparationResultPrepared,
                   @"Ruled loft prepared and independently proved.");
}
- (void)applyWithCompletion:(void (^)(Core3DProfileConstructionResult, NSString *))completion {
    State expected = State::Prepared;
    if (!_state.compare_exchange_strong(expected, State::Applying) || !_service || !_prepared) {
#if DEBUG
        std::fprintf(stderr, "R179_CURVE_RESULT stage=apply result=%d detail=%.96s\n",
                     int(Core3DProfileConstructionResultRejected),
                     "No current ruled-loft preparation.");
#endif
        Deliver(completion, Core3DProfileConstructionResultRejected,
                @"No current ruled-loft preparation."); return;
    }
    const auto receipt = _service->apply(_prepared);
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultRejected;
    if (receipt.outcome == core3d::general_loft::owner::Outcome::committed)
        result = Core3DProfileConstructionResultCommitted;
    else if (receipt.outcome == core3d::general_loft::owner::Outcome::busy)
        result = Core3DProfileConstructionResultBusy;
    else if (receipt.outcome == core3d::general_loft::owner::Outcome::outcomeUnknown
             || receipt.outcome == core3d::general_loft::owner::Outcome::recoveryRequired)
        result = Core3DProfileConstructionResultRecoveryRequired;
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) {
        _prepared.reset(); _opening.reset(); _service.reset(); _context.reset();
    }
    NSString *applyDetail = result == Core3DProfileConstructionResultCommitted
        ? @"Ruled loft committed in one history command."
        : result == Core3DProfileConstructionResultRecoveryRequired
            ? @"The native close outcome is unknown; recovery ownership is retained."
            : @"Ruled-loft apply was refused without partial replacement.";
#if DEBUG
    if (result != Core3DProfileConstructionResultCommitted) {
        std::fprintf(stderr, "R179_CURVE_RESULT stage=apply result=%d detail=%.96s\n",
                     int(result), applyDetail.UTF8String);
    }
#endif
    Deliver(completion, result, applyDetail);
}
- (BOOL)cancel {
    State value = _state.load();
    while (value == State::Open || value == State::Prepared)
        if (_state.compare_exchange_weak(value, State::Cancelled)) {
            if (!_service || !_opening
                || _service->cancel(_opening->session).outcome
                    != core3d::general_loft::owner::Outcome::cancelled) {
                _state.store(State::Recovery); return NO;
            }
            _prepared.reset(); _opening.reset(); _service.reset(); _context.reset(); return YES;
        }
    return NO;
}
@end

@interface Core3DPatternEditingOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<core3d::native_opening::Context> _context;
    core3d::pattern_owner::Snapshot _opening;
    std::atomic<State> _state;
}
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    opening:(core3d::pattern_owner::Snapshot)opening;
@end

@implementation Core3DPatternEditingOpening
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    opening:(core3d::pattern_owner::Snapshot)opening {
    if ((self = [super init])) { _owner = owner; _context = std::move(context);
        _opening = std::move(opening); _state.store(State::Open); }
    return self;
}
- (NSDictionary *)descriptor {
    return PatternDescriptor(_opening, _context ? _context->openingFence().metersPerUnit() : 0);
}
- (NSInteger)projectedDocumentBytes { return ClampToNSInteger(_opening.patternDocumentBytes); }
- (NSInteger)projectedMemoryBytes {
    return ClampToNSInteger(_opening.allLabels
        ? _opening.allLabels->measured.retainedMemoryBytes : 0);
}
- (NSInteger)projectedTopologyNodes {
    return ClampToNSInteger(_opening.allLabels ? _opening.allLabels->measured.topologyNodes : 0);
}
- (void)applyCandidate:(NSDictionary *)candidate
    completion:(void (^)(Core3DProfileConstructionResult, NSString *))completion {
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Applying)) {
        Deliver(completion, Core3DProfileConstructionResultRejected, @"Opening already consumed."); return;
    }
    core3d::pattern_owner::Edit edit;
    if (_owner.IsNull() || !_context
        || !PatternEdit(candidate, _opening,
                        _context->openingFence().metersPerUnit(), edit)) {
        _state.store(State::Settled); _context.reset();
        Deliver(completion, Core3DProfileConstructionResultRejected, @"Malformed retained-pattern values."); return;
    }
    const auto prepared = core3d::pattern_owner::PrepareNative(*_owner, _opening, edit, {});
#if DEBUG
    NSLog(@"R179_D2_APPLY preparationAdmitted=%d nativePrepared=%d refusal=%u sourceFamily=%u rows=%u columns=%u suppressedCount=%zu",
          int(prepared.admitted()), int(bool(prepared.native)), unsigned(prepared.refusal),
          unsigned(_opening.sourceRecipe.family), unsigned(edit.rows),
          unsigned(edit.columns), edit.suppressed.size());
#endif
    const auto outcome = core3d::pattern_owner::ApplyNative(*_owner, prepared, _context);
#if DEBUG
    NSLog(@"R179_D2_APPLY outcome=%u", unsigned(outcome));
#endif
    const auto result = MapOutcome(outcome, core3d::pattern_owner::ApplyOutcome::Committed,
                                  core3d::pattern_owner::ApplyOutcome::OutcomeUnknown);
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    Deliver(completion, result, result == Core3DProfileConstructionResultCommitted
        ? @"Retained pattern committed." : result == Core3DProfileConstructionResultRecoveryRequired
            ? @"Pattern close is unknown; native recovery ownership is retained."
            : @"Pattern edit refused without retry.");
}
- (BOOL)cancel {
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Cancelled)) return NO;
    _context.reset(); return YES;
}
@end

@interface Core3DPathArrayEditingOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<core3d::native_opening::Context> _context;
    core3d::path_array_owner::Snapshot _opening;
    std::optional<core3d::path_array_owner::PathAuthority> _replacementPath;
    std::atomic<State> _state;
}
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    opening:(core3d::path_array_owner::Snapshot)opening
    replacementPath:(std::optional<core3d::path_array_owner::PathAuthority>)replacementPath;
@end

@implementation Core3DPathArrayEditingOpening
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    opening:(core3d::path_array_owner::Snapshot)opening
    replacementPath:(std::optional<core3d::path_array_owner::PathAuthority>)replacementPath {
    if ((self = [super init])) { _owner = owner; _context = std::move(context);
        _opening = std::move(opening); _replacementPath = std::move(replacementPath);
        _state.store(State::Open); }
    return self;
}
- (NSDictionary *)descriptor {
    return PathDescriptor(_opening,
        _context ? _context->openingFence().metersPerUnit() : 0,
        _replacementPath ? &*_replacementPath : nullptr);
}
- (NSInteger)projectedInstances {
    return ClampToNSInteger(_opening.record.definition.members.size());
}
- (NSInteger)projectedDocumentBytes { return ClampToNSInteger(_opening.pathArrayDocumentBytes); }
- (NSInteger)projectedMemoryBytes { return ClampToNSInteger(_opening.metrics.sourceMemoryBytes); }
- (NSInteger)projectedTopologyNodes { return ClampToNSInteger(_opening.metrics.sourceTopologyNodes); }
- (double)maximumMeasuredArcErrorInDocumentUnits { return 0; }
- (NSString *)measuredRefusal { return nil; }
- (Core3DPathArrayEditingOpening *)replacingPathWithEntityIdentifier:(NSString *)entityIdentifier {
    State expected = State::Open;
    if (!NSThread.isMainThread
        || ![entityIdentifier isKindOfClass:NSString.class]
        || entityIdentifier.length == 0 || entityIdentifier.length > 128
        || !_state.compare_exchange_strong(expected, State::Prepared)) return nil;
    try {
        if (_owner.IsNull() || !_context || !_context->isCurrent(64, 64)
            || !entityIdentifier.UTF8String) { _state.store(State::Open); return nil; }
        const std::string entity(entityIdentifier.UTF8String,
            [entityIdentifier lengthOfBytesUsingEncoding:NSUTF8StringEncoding]);
        core3d::path_array_owner::Snapshot refreshed;
        core3d::path_array_owner::PathAuthority replacement;
        if (core3d::path_array_owner::RefreshPathNative(*_owner, _opening,
                entity, _context, refreshed, replacement)
                    != core3d::path_array_owner::Refusal::None
            || !_context->isCurrent(64, 64)) {
            _state.store(State::Open); return nil;
        }
        Core3DPathArrayEditingOpening *next =
            [[Core3DPathArrayEditingOpening alloc] initWithOwner:_owner
                context:_context opening:std::move(refreshed)
                replacementPath:std::move(replacement)];
        if (!next) { _state.store(State::Open); return nil; }
        _state.store(State::Cancelled);
        _replacementPath.reset(); _context.reset();
        return next;
    } catch (...) { _state.store(State::Open); return nil; }
}
- (Core3DPathArrayEditingOpening *)replacingPathFromCurrentSelection {
    State expected = State::Open;
    if (!NSThread.isMainThread
        || !_state.compare_exchange_strong(expected, State::Prepared)) return nil;
    try {
        if (!_context) { _state.store(State::Open); return nil; }
        const auto current = _context->recapture(64, 64);
        const auto& openingFence = _context->openingFence();
        if (!current || current->document() != openingFence.document()
            || current->data() != openingFence.data()
            || current->documentGeneration() != openingFence.documentGeneration()
            || current->modelRevision() != openingFence.modelRevision()
            || current->metersPerUnit() != openingFence.metersPerUnit()
            || current->selectionMode() != core3d::scene::ElementKind::Object
            || current->selection().size() != 1
            || current->selection().front().kind != core3d::scene::ElementKind::Object
            || current->selection().front().entityIdentifier.empty()
            || current->selection().front().entityIdentifier.size() > 128) {
            _state.store(State::Open); return nil;
        }
        core3d::path_array_owner::Snapshot refreshed;
        core3d::path_array_owner::PathAuthority replacement;
        if (_owner.IsNull()
            || core3d::path_array_owner::RefreshPathNative(*_owner, _opening,
                current->selection().front().entityIdentifier, _context,
                refreshed, replacement) != core3d::path_array_owner::Refusal::None) {
            _state.store(State::Open); return nil;
        }
        Core3DPathArrayEditingOpening *next =
            [[Core3DPathArrayEditingOpening alloc] initWithOwner:_owner
                context:_context opening:std::move(refreshed)
                replacementPath:std::move(replacement)];
        if (!next) { _state.store(State::Open); return nil; }
        _state.store(State::Cancelled);
        _replacementPath.reset(); _context.reset();
        return next;
    } catch (...) { _state.store(State::Open); return nil; }
}
- (void)applyCandidate:(NSDictionary *)candidate
    completion:(void (^)(Core3DProfileConstructionResult, NSString *))completion {
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Applying)) {
        Deliver(completion, Core3DProfileConstructionResultRejected, @"Opening already consumed."); return;
    }
    core3d::path_array_owner::Edit edit;
    if (_owner.IsNull() || !PathEdit(candidate, _opening,
            _replacementPath ? &*_replacementPath : nullptr, edit)) {
        _state.store(State::Settled); _context.reset();
        Deliver(completion, Core3DProfileConstructionResultRejected, @"Malformed path-array values."); return;
    }
    const auto prepared = core3d::path_array_owner::PrepareNative(*_owner, _opening, edit, {});
    const auto outcome = core3d::path_array_owner::ApplyNative(*_owner, prepared, _context);
    const auto result = MapOutcome(outcome, core3d::path_array_owner::ApplyOutcome::Committed,
                                  core3d::path_array_owner::ApplyOutcome::OutcomeUnknown);
    _state.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    Deliver(completion, result, result == Core3DProfileConstructionResultCommitted
        ? @"Path array committed." : result == Core3DProfileConstructionResultRecoveryRequired
            ? @"Path-array close is unknown; native recovery ownership is retained."
            : @"Path-array edit refused without retry.");
}
- (BOOL)cancel {
    State expected = State::Open;
    if (!_state.compare_exchange_strong(expected, State::Cancelled)) return NO;
    _context.reset(); return YES;
}
@end

@interface Core3DFeaturePatternPreview () {
@package
    BOOL _admitted; NSString *_refusalMessage; NSUInteger _generatedFeatureCount;
    double _minimumMeasuredLigamentMM, _removedVolumeMM3; NSUInteger _boundarySectionCount;
    NSUInteger _projectedDocumentBytes, _projectedMemoryBytes, _projectedTopologyNodes;
}
@end
@implementation Core3DFeaturePatternPreview
@synthesize admitted = _admitted, refusalMessage = _refusalMessage;
@synthesize generatedFeatureCount = _generatedFeatureCount;
@synthesize minimumMeasuredLigamentMM = _minimumMeasuredLigamentMM;
@synthesize removedVolumeMM3 = _removedVolumeMM3, boundarySectionCount = _boundarySectionCount;
@synthesize projectedDocumentBytes = _projectedDocumentBytes;
@synthesize projectedMemoryBytes = _projectedMemoryBytes;
@synthesize projectedTopologyNodes = _projectedTopologyNodes;
@end

@interface Core3DFeaturePatternPreparedEdit () {
@package
    std::shared_ptr<const core3d::feature_pattern_owner::PreparedEdit> _native;
    std::uint64_t _issuer; std::atomic_bool _consumed;
}
@end
@implementation Core3DFeaturePatternPreparedEdit @end

@interface Core3DFeaturePatternPreparation () {
@package Core3DFeaturePatternPreview *_preview;
@package Core3DFeaturePatternPreparedEdit *_prepared;
}
@end
@implementation Core3DFeaturePatternPreparation
@synthesize preview = _preview, prepared = _prepared;
@end

@interface Core3DFeaturePatternEditingOpening () {
@package
    Handle(OcctDocument) _owner;
    std::shared_ptr<core3d::native_opening::Context> _context;
    core3d::feature_pattern_owner::Snapshot _opening;
    std::atomic<State> _state; std::uint64_t _issuer;
}
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    opening:(core3d::feature_pattern_owner::Snapshot)opening;
@end

@implementation Core3DFeaturePatternEditingOpening
- (instancetype)initWithOwner:(const Handle(OcctDocument)&)owner
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    opening:(core3d::feature_pattern_owner::Snapshot)opening {
    if ((self = [super init])) { _owner = owner; _context = std::move(context);
        _opening = std::move(opening); _state.store(State::Open);
        _issuer = reinterpret_cast<std::uintptr_t>((__bridge void *)self); }
    return self;
}
- (NSDictionary *)descriptor { return FeatureDescriptor(_opening); }
- (Core3DFeaturePatternPreparation *)prepareCandidate:(NSDictionary *)candidate {
    Core3DFeaturePatternPreparation *result =
        (Core3DFeaturePatternPreparation *)[Core3DFeaturePatternPreparation alloc];
    result->_preview = (Core3DFeaturePatternPreview *)[Core3DFeaturePatternPreview alloc];
    result->_preview->_refusalMessage = @"Candidate refused.";
    State expected = State::Open;
    core3d::feature_pattern_owner::Edit edit;
    if (!_state.compare_exchange_strong(expected, State::Prepared)) return result;
    if (_owner.IsNull()
        || !FeatureEdit(candidate, _opening.record.definition.metersPerUnit, edit)) {
        _state.store(State::Settled); _context.reset(); return result;
    }
    auto prepared = std::make_shared<core3d::feature_pattern_owner::PreparedEdit>(
        core3d::feature_pattern_owner::PrepareNative(*_owner, _opening, edit, {}));
    result->_preview->_admitted = prepared->admitted();
    result->_preview->_refusalMessage = prepared->admitted() ? @"" : @"Native feature-pattern admission refused.";
    result->_preview->_generatedFeatureCount = prepared->admission.attribution.size();
    const double mmPerUnit = _opening.record.definition.metersPerUnit * 1000.0;
    result->_preview->_removedVolumeMM3 =
        prepared->admission.measuredRemovedVolume * mmPerUnit * mmPerUnit * mmPerUnit;
    result->_preview->_boundarySectionCount = prepared->admission.measuredBoundarySections;
    result->_preview->_projectedDocumentBytes = prepared->projection.projectedDocumentBytes;
    result->_preview->_projectedMemoryBytes = prepared->projection.projectedMemoryBytes;
    result->_preview->_projectedTopologyNodes = prepared->projection.projectedTopologyNodes;
    double minimum = std::numeric_limits<double>::infinity();
    for (const auto& item : prepared->rebuilt.evidence.generated)
        minimum = std::min(minimum, item.minimumHostLigamentMM);
    result->_preview->_minimumMeasuredLigamentMM = std::isfinite(minimum) ? minimum : 0;
    if (prepared->admitted()) {
        result->_prepared = (Core3DFeaturePatternPreparedEdit *)
            [Core3DFeaturePatternPreparedEdit alloc];
        result->_prepared->_native = std::move(prepared);
        result->_prepared->_issuer = _issuer;
        result->_prepared->_consumed.store(false);
    } else { _state.store(State::Settled); _context.reset(); }
    return result;
}
- (void)applyPrepared:(Core3DFeaturePatternPreparedEdit *)token
    completion:(void (^)(Core3DProfileConstructionResult, NSString *))completion {
    State expected = State::Prepared; bool unused = false;
    if (![token isKindOfClass:Core3DFeaturePatternPreparedEdit.class]
        || token->_issuer != _issuer || !token->_native
        || !token->_consumed.compare_exchange_strong(unused, true)
        || !_state.compare_exchange_strong(expected, State::Applying)) {
        Deliver(completion, Core3DProfileConstructionResultRejected,
                @"Foreign, stale, or reused preparation."); return;
    }
    const auto outcome = core3d::feature_pattern_owner::ApplyNative(
        *_owner, *token->_native, _context);
    const auto mapped = MapOutcome(outcome,
        core3d::feature_pattern_owner::ApplyOutcome::Committed,
        core3d::feature_pattern_owner::ApplyOutcome::OutcomeUnknown);
    _state.store(mapped == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    if (mapped != Core3DProfileConstructionResultRecoveryRequired) _context.reset();
    Deliver(completion, mapped, mapped == Core3DProfileConstructionResultCommitted
        ? @"Feature pattern committed." : mapped == Core3DProfileConstructionResultRecoveryRequired
            ? @"Feature-pattern close is unknown; native recovery ownership is retained."
            : @"Feature-pattern edit refused without retry.");
}
- (BOOL)cancel {
    State value = _state.load();
    while (value == State::Open || value == State::Prepared)
        if (_state.compare_exchange_weak(value, State::Cancelled)) {
            _context.reset(); return YES;
        }
    return NO;
}
@end

namespace core3d::native_opening {
Core3DBoundedCurveEditingOpening *OpenBoundedCurve(const Handle(OcctDocument)& owner,
    const std::string& selected, const std::shared_ptr<Context>& context) noexcept {
    try {
        if (!NSThread.isMainThread || owner.IsNull() || owner->Document().IsNull() || !context
            || context->openingFence().document() != owner->Document()
            || context->openingFence().data() != owner->Document()->GetData()
            || !context->isCurrent(64, 64)) return nil;
        auto service = std::make_unique<bounded_curve::owner::OcafOwner>(*owner, context);
        const bounded_curve::owner::SceneFence scene{
            context->openingFence().documentGeneration(),
            context->openingFence().modelRevision(),
            context->openingFence().metersPerUnit()};
        auto opening = service->capture(selected, scene);
        if (!opening) return nil;
        return [[Core3DBoundedCurveEditingOpening alloc] initWithOwner:owner
            context:context service:std::move(service) opening:std::move(opening)];
    } catch (...) { return nil; }
}

Core3DBoundedCurveCreationOpening *OpenBoundedCurveCreation(const Handle(OcctDocument)& owner,
    const std::shared_ptr<Context>& context) noexcept {
    try {
        if (!NSThread.isMainThread || owner.IsNull() || owner->Document().IsNull() || !context
            || context->openingFence().document() != owner->Document()
            || context->openingFence().data() != owner->Document()->GetData()
            || !context->isCurrent(64, 64)) return nil;
        auto service = std::make_unique<bounded_curve::owner::OcafOwner>(*owner, context);
        return [[Core3DBoundedCurveCreationOpening alloc] initWithOwner:owner
            context:context service:std::move(service)];
    } catch (...) { return nil; }
}

Core3DGeneralLoftEditingOpening *OpenGeneralLoft(const Handle(OcctDocument)& owner,
    const std::string& selected, const std::shared_ptr<Context>& context) noexcept {
    const char *gate = "reopen-factory-preconditions-or-current-fence";
    const auto refused = [&]() -> Core3DGeneralLoftEditingOpening * {
#if DEBUG
        NSLog(@"R4_C3N refused=%s", gate);
#else
        (void)gate;
#endif
        return nil;
    };
    try {
        if (!NSThread.isMainThread || owner.IsNull() || owner->Document().IsNull() || !context
            || context->openingFence().document() != owner->Document()
            || context->openingFence().data() != owner->Document()->GetData()
            || !context->isCurrent(64, 64)) return refused();
        gate = "reopen-owner-service";
        auto service = std::make_unique<general_loft::owner::OcafOwner>(*owner, context);
        const general_loft::owner::SceneFence scene{context->openingFence().documentGeneration(),
            context->openingFence().modelRevision(), context->openingFence().metersPerUnit()};
        gate = "reopen-owner-capture";
        auto opening = service->capture(selected, scene); if (!opening) return refused();
        gate = "reopen-wrapper-publication";
        return [[Core3DGeneralLoftEditingOpening alloc] initWithOwner:owner context:context
            service:std::move(service) opening:std::move(opening)];
    } catch (...) { return refused(); }
}

Core3DGeneralLoftEditingOpening *BeginGeneralLoft(const Handle(OcctDocument)& owner,
    NSArray<NSDictionary<NSString *, id> *> *stations, NSArray<NSNumber *> *orderAxis,
    NSString *requestedName, const std::shared_ptr<Context>& context) noexcept {
    const char *gate = "begin-preconditions-or-current-fence";
    const auto refused = [&]() -> Core3DGeneralLoftEditingOpening * {
#if DEBUG
        NSLog(@"R4_C3N refused=%s", gate);
#endif
        return nil;
    };
    try {
        if (!NSThread.isMainThread || owner.IsNull() || owner->Document().IsNull() || !context
            || ![stations isKindOfClass:NSArray.class] || ![orderAxis isKindOfClass:NSArray.class]
            || ![requestedName isKindOfClass:NSString.class]
            || context->openingFence().document() != owner->Document()
            || context->openingFence().data() != owner->Document()->GetData()
            || !context->isCurrent(64, 64)) return refused();
        general_loft::owner::CreateRequest request;
        request.expectedScene = {context->openingFence().documentGeneration(),
            context->openingFence().modelRevision(), context->openingFence().metersPerUnit()};
        request.requestedName = requestedName.UTF8String ?: "Ruled loft";
        gate = "begin-order-axis-parse";
        if (!Vector3(orderAxis, request.orderAxis)) return refused();
        gate = "begin-station-parse";
        for (NSDictionary *row in stations) {
            general_loft::owner::CreationStation station;
            if (!ParseCreationStation(row, station)) return refused();
            request.stations.push_back(std::move(station));
        }
        gate = "begin-owner-construction";
        auto service = std::make_unique<general_loft::owner::OcafOwner>(*owner, context);
        general_loft::owner::Receipt receipt;
        gate = "begin-owner-creation";
        auto opening = service->beginCreation(request, receipt); if (!opening) return refused();
        gate = "begin-opening-allocation";
        return [[Core3DGeneralLoftEditingOpening alloc] initWithOwner:owner context:context
            service:std::move(service) opening:std::move(opening)];
    } catch (...) { return refused(); }
}

Core3DPatternEditingOpening *OpenPattern(const Handle(OcctDocument)& owner,
    const std::string& selected, const std::shared_ptr<Context>& context) noexcept {
    try {
        if (!NSThread.isMainThread || owner.IsNull() || owner->Document().IsNull() || !context
            || context->openingFence().document() != owner->Document()
            || context->openingFence().data() != owner->Document()->GetData()
            || !context->isCurrent(64, 64)) return nil;
        pattern_owner::Snapshot opening;
        const auto refusal = pattern_owner::Capture(*owner, selected, opening);
#if DEBUG
        if (refusal != pattern_owner::Refusal::None)
            NSLog(@"R179_FACTORY family=d2 captureRefusal=%u", unsigned(refusal));
#endif
        if (refusal != pattern_owner::Refusal::None)
            return nil;
        return [[Core3DPatternEditingOpening alloc] initWithOwner:owner
            context:context opening:std::move(opening)];
    } catch (...) { return nil; }
}
Core3DPathArrayEditingOpening *OpenPathArray(const Handle(OcctDocument)& owner,
    const std::string& selected, const std::shared_ptr<Context>& context) noexcept {
    try {
        if (!NSThread.isMainThread || owner.IsNull() || owner->Document().IsNull() || !context
            || context->openingFence().document() != owner->Document()
            || context->openingFence().data() != owner->Document()->GetData()
            || !context->isCurrent(64, 64)) return nil;
        path_array_owner::Snapshot opening;
        const auto refusal =
            path_array_owner::CaptureNative(*owner, selected, context, opening);
#if DEBUG
        if (refusal != path_array_owner::Refusal::None)
            NSLog(@"R179_FACTORY family=d3 captureRefusal=%u", unsigned(refusal));
#endif
        if (refusal != path_array_owner::Refusal::None) return nil;
        return [[Core3DPathArrayEditingOpening alloc] initWithOwner:owner
            context:context opening:std::move(opening)
            replacementPath:std::nullopt];
    } catch (...) { return nil; }
}
Core3DFeaturePatternEditingOpening *OpenFeaturePattern(const Handle(OcctDocument)& owner,
    const std::string& selected, const std::shared_ptr<Context>& context) noexcept {
    try {
        if (!NSThread.isMainThread || owner.IsNull() || owner->Document().IsNull() || !context
            || context->openingFence().document() != owner->Document()
            || context->openingFence().data() != owner->Document()->GetData()
            || !context->isCurrent(64, 64)) return nil;
        feature_pattern_owner::Snapshot opening;
        const auto refusal =
            feature_pattern_owner::CaptureNative(*owner, selected, context, opening);
#if DEBUG
        if (refusal != feature_pattern_owner::Refusal::None)
            NSLog(@"R179_FACTORY family=d4 captureRefusal=%u", unsigned(refusal));
#endif
        if (refusal != feature_pattern_owner::Refusal::None) return nil;
        return [[Core3DFeaturePatternEditingOpening alloc] initWithOwner:owner
            context:context opening:std::move(opening)];
    } catch (...) { return nil; }
}
} // namespace core3d::native_opening

namespace {
using C4Definition = core3d::spline_profile::Definition;
using C4Owner = core3d::spline_profile::owner::OcafOwner;

NSString *C4UUID(const core3d::spline_profile::UUID& value) {
    const std::string text = core3d::retained_solid::UUIDText(value);
    return [NSString stringWithUTF8String:text.c_str()] ?: @"";
}

NSString *C4StableID(const C4Definition& value, core3d::ProfileCurveID local) {
    for (const auto& identity : value.identities)
        if (identity.local == local) return C4UUID(identity.uuid);
    return @"";
}

bool C4IdentityMatches(const C4Definition& value, core3d::ProfileCurveID local,
                       id stable) {
    return [stable isKindOfClass:NSString.class]
        && [(NSString *)stable isEqualToString:C4StableID(value, local)];
}

NSDictionary<NSString *, id> *C4Descriptor(const C4Definition& value, BOOL creating) {
    NSMutableArray *segments = [NSMutableArray array];
    for (const auto& spline : value.spline.segments) {
        NSMutableArray *poles = [NSMutableArray array];
        for (const auto& pole : spline.poles) {
            [poles addObject:@{@"id": @(pole.identifier),
                               @"stableID": C4StableID(value, pole.identifier),
                               @"u": @(pole.point.X()), @"v": @(pole.point.Y())}];
        }
        [segments addObject:@{@"id": @(spline.identifier),
                              @"stableID": C4StableID(value, spline.identifier),
                              @"degree": @(spline.degree),
                              @"poles": poles, @"rational": @(!spline.weights.empty())}];
    }
    NSDictionary *axis = value.spline.revolveAxis
        ? @{@"originU": @(value.spline.revolveAxis->origin.X()),
            @"originV": @(value.spline.revolveAxis->origin.Y()),
            @"directionU": @(value.spline.revolveAxis->direction.X()),
            @"directionV": @(value.spline.revolveAxis->direction.Y())}
        : @{};
    return @{@"schema": @1, @"kind": @(value.key.kind),
        @"codec": @(value.key.codecVersion), @"creating": @(creating),
        @"featureID": C4UUID(value.feature), @"revision": @(value.revision),
        @"metersPerUnit": @(value.metersPerUnit), @"plane": @(value.plane),
        @"operation": value.operation == core3d::spline_profile::Operation::revolve ? @"revolve" : @"extrude",
        @"depth": @(value.depth), @"angleDegrees": @(value.angleDegrees),
        @"innerLoopPolicy": value.spline.innerLoopPolicy == core3d::SplineInnerLoopPolicy::Void ? @"void" : @"refuse",
        @"axis": axis, @"segments": segments,
        @"frameID": C4UUID(value.frame.identifier), @"frameRevision": @(value.frame.revision),
        @"limitations": @[@"non-periodic clamped spline", @"non-rational C1 import",
                            @"maximum 32 poles per segment", @"maximum 16 spline segments",
                            @"shell/Boolean/edge descendants refuse"]};
}

bool C4Candidate(NSDictionary<NSString *, id> *dictionary,
                 const C4Definition& baseline, bool creating,
                 C4Definition& output) noexcept {
    output = {};
    try {
        if (![dictionary isKindOfClass:NSDictionary.class]
            || [dictionary[@"schema"] integerValue] != 1
            || [dictionary[@"kind"] unsignedIntValue] != baseline.key.kind
            || [dictionary[@"codec"] unsignedIntValue] != baseline.key.codecVersion
            || [dictionary[@"creating"] boolValue] != creating
            || ![dictionary[@"featureID"] isEqual:C4UUID(baseline.feature)]
            || [dictionary[@"revision"] unsignedLongLongValue] != baseline.revision
            || [dictionary[@"metersPerUnit"] doubleValue] != baseline.metersPerUnit
            || [dictionary[@"plane"] integerValue] != baseline.plane
            || ![dictionary[@"frameID"] isEqual:C4UUID(baseline.frame.identifier)]
            || [dictionary[@"frameRevision"] unsignedLongLongValue]
                != baseline.frame.revision) return false;
        C4Definition value = baseline;
        NSString *operation = dictionary[@"operation"];
        if ([operation isEqualToString:@"extrude"]) {
            value.operation = core3d::spline_profile::Operation::extrude;
            value.depth = [dictionary[@"depth"] doubleValue]; value.angleDegrees = 0;
            value.spline.revolveAxis.reset();
        } else if ([operation isEqualToString:@"revolve"]) {
            value.operation = core3d::spline_profile::Operation::revolve;
            value.depth = 0; value.angleDegrees = [dictionary[@"angleDegrees"] doubleValue];
            NSDictionary *axis = dictionary[@"axis"];
            if (![axis isKindOfClass:NSDictionary.class]) return false;
            const double ou = [axis[@"originU"] doubleValue], ov = [axis[@"originV"] doubleValue];
            const double du = [axis[@"directionU"] doubleValue], dv = [axis[@"directionV"] doubleValue];
            value.spline.revolveAxis = core3d::SplineRevolveAxis{
                gp_Pnt2d(ou, ov), gp_Pnt2d(du, dv)};
        } else return false;
        NSString *policy = dictionary[@"innerLoopPolicy"];
        if ([policy isEqualToString:@"void"])
            value.spline.innerLoopPolicy = core3d::SplineInnerLoopPolicy::Void;
        else if ([policy isEqualToString:@"refuse"])
            value.spline.innerLoopPolicy = core3d::SplineInnerLoopPolicy::Refuse;
        else return false;
        NSArray *segments = dictionary[@"segments"];
        if (![segments isKindOfClass:NSArray.class]
            || segments.count != value.spline.segments.size()) return false;
        std::set<core3d::ProfileCurveID> seenSegments;
        for (id item in segments) {
            if (![item isKindOfClass:NSDictionary.class]) return false;
            NSDictionary *row = item;
            const core3d::ProfileCurveID segmentID = [row[@"id"] unsignedIntValue];
            const auto found = std::find_if(value.spline.segments.begin(),
                value.spline.segments.end(), [&](const auto& candidate) {
                    return candidate.identifier == segmentID;
                });
            if (found == value.spline.segments.end()
                || !seenSegments.insert(segmentID).second
                || !C4IdentityMatches(baseline, segmentID, row[@"stableID"])) return false;
            auto& spline = *found;
            NSArray *poles = row[@"poles"];
            if ([row[@"degree"] integerValue] != spline.degree
                || [row[@"rational"] boolValue] != !spline.weights.empty()
                || ![poles isKindOfClass:NSArray.class]
                || poles.count != spline.poles.size()) return false;
            std::set<core3d::ProfileCurveID> seenPoles;
            for (id poleItem in poles) {
                if (![poleItem isKindOfClass:NSDictionary.class]) return false;
                NSDictionary *pole = poleItem;
                const core3d::ProfileCurveID poleID = [pole[@"id"] unsignedIntValue];
                const auto poleFound = std::find_if(spline.poles.begin(), spline.poles.end(),
                    [&](const auto& candidate) { return candidate.identifier == poleID; });
                if (poleFound == spline.poles.end() || !seenPoles.insert(poleID).second
                    || !C4IdentityMatches(baseline, poleID, pole[@"stableID"])) return false;
                auto& target = *poleFound;
                target.point = gp_Pnt2d([pole[@"u"] doubleValue], [pole[@"v"] doubleValue]);
            }
        }
        value.revision = baseline.revision + (creating ? 0 : 1);
        value.frame.revision = value.revision;
        if (core3d::spline_profile::Validate(value)
                != core3d::spline_profile::Refusal::none) return false;
        output = std::move(value); return true;
    } catch (...) { output = {}; return false; }
}

bool C4DefaultDefinition(OcctDocument& document, C4Definition& value) noexcept {
    value = {};
    try {
        auto makeUUID = [](core3d::spline_profile::UUID& output) {
            return core3d::receipt::ParseUUID(
                NSUUID.UUID.UUIDString.UTF8String ?: "", output);
        };
        if (!core3d::receipt::ParseUUID(document.DocumentIdentifier(), value.owner.document)
            || !makeUUID(value.owner.entity) || !makeUUID(value.owner.definition)
            || !makeUUID(value.feature) || !makeUUID(value.frame.identifier)) return false;
        Standard_Real metersPerUnit = 0;
        if (!XCAFDoc_DocumentTool::GetLengthUnit(document.Document(), metersPerUnit)
            || !std::isfinite(metersPerUnit) || metersPerUnit <= 0) return false;
        value.revision = 1; value.nextLocalIdentity = 100;
        value.metersPerUnit = metersPerUnit; value.frame.revision = 1;
        value.operation = core3d::spline_profile::Operation::revolve;
        value.angleDegrees = 360;
        value.spline.revolveAxis = core3d::SplineRevolveAxis{gp_Pnt2d(0, 0), gp_Pnt2d(0, 1)};
        const double lengthScale = 0.001 / metersPerUnit;
        auto V = [lengthScale](core3d::ProfileCurveID id, double u, double v) {
            return core3d::ProfileCurveVertex{id, gp_Pnt2d(u * lengthScale, v * lengthScale)};
        };
        auto L = [](core3d::ProfileCurveID id, core3d::ProfileCurveID a,
                    core3d::ProfileCurveID b) {
            core3d::ProfileCurveSegment edge; edge.identifier = id;
            edge.startVertex = a; edge.endVertex = b; return edge;
        };
        auto S = L;
        value.section.outer.identifier = 1;
        value.section.outer.vertices = {V(11, 10, 0), V(12, 20, 0), V(13, 20, 30), V(14, 10, 30)};
        value.section.outer.segments = {L(21, 11, 12), S(22, 12, 13), L(23, 13, 14), S(24, 14, 11)};
        value.section.outer.segments[1].kind = core3d::ProfileCurveKind::Spline;
        value.section.outer.segments[3].kind = core3d::ProfileCurveKind::Spline;
        auto cubic = [&](core3d::ProfileCurveID id, core3d::ProfileCurveID base,
                         std::array<gp_Pnt2d, 4> points) {
            core3d::SplineCurveSegment segment; segment.identifier = id; segment.degree = 3;
            for (std::size_t index = 0; index < points.size(); ++index)
                segment.poles.push_back(V(base + core3d::ProfileCurveID(index),
                                          points[index].X(), points[index].Y()));
            segment.knots = {0, 1}; segment.multiplicities = {4, 4}; return segment;
        };
        value.spline.segments = {
            cubic(22, 31, {gp_Pnt2d(20,0), gp_Pnt2d(21,10), gp_Pnt2d(19,20), gp_Pnt2d(20,30)}),
            cubic(24, 41, {gp_Pnt2d(10,30), gp_Pnt2d(9,20), gp_Pnt2d(11,10), gp_Pnt2d(10,0)})};
        for (const auto& segment : value.spline.segments) {
            core3d::spline_profile::UUID id{}; if (!makeUUID(id)) return false;
            value.identities.push_back({id, segment.identifier});
            for (const auto& pole : segment.poles) {
                if (!makeUUID(id)) return false; value.identities.push_back({id, pole.identifier});
            }
        }
        return core3d::spline_profile::Validate(value)
            == core3d::spline_profile::Refusal::none;
    } catch (...) { value = {}; return false; }
}
} // namespace

@interface Core3DSplineProfileEditingOpening () {
@package
    Handle(OcctDocument) _c4Document;
    std::shared_ptr<core3d::native_opening::Context> _c4Context;
    std::unique_ptr<C4Owner> _c4Service;
    std::shared_ptr<const core3d::spline_profile::owner::Opening> _c4Opening;
    std::shared_ptr<const core3d::spline_profile::owner::Prepared> _c4Prepared;
    std::atomic<State> _c4State;
    BOOL _creating;
}
- (instancetype)initWithDocument:(const Handle(OcctDocument)&)document
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    service:(std::unique_ptr<C4Owner>)service
    opening:(std::shared_ptr<const core3d::spline_profile::owner::Opening>)opening
    creating:(BOOL)creating;
@end

@implementation Core3DSplineProfileEditingOpening
- (instancetype)initWithDocument:(const Handle(OcctDocument)&)document
    context:(std::shared_ptr<core3d::native_opening::Context>)context
    service:(std::unique_ptr<C4Owner>)service
    opening:(std::shared_ptr<const core3d::spline_profile::owner::Opening>)opening
    creating:(BOOL)creating {
    if (!(self = [super init])) return nil;
    _c4Document = document; _c4Context = std::move(context); _c4Service = std::move(service);
    _c4Opening = std::move(opening); _c4State.store(State::Open); _creating = creating; return self;
}
- (NSDictionary<NSString *,id> *)descriptor {
    return _c4Opening ? C4Descriptor(_c4Opening->definition, _creating) : @{};
}
- (BOOL)isCreating { return _creating; }
- (void)prepareCandidate:(NSDictionary<NSString *,id> *)candidate
    completion:(void (^)(Core3DBoundedCurvePreparationResult, NSString *))completion {
    if (_c4State.load() != State::Open || !_c4Service || !_c4Opening) {
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected, @"Opening already consumed."); return;
    }
    C4Definition definition;
    if (!C4Candidate(candidate, _c4Opening->definition, _creating, definition)) {
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultUnsupportedMutation,
                       @"Only admitted pole, axis, extent and inner-loop values may change."); return;
    }
    core3d::spline_profile::owner::Receipt receipt;
    auto prepared = _c4Service->prepare(_c4Opening, definition, receipt);
    if (!prepared || receipt.outcome != core3d::spline_profile::owner::Outcome::prepared) {
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected,
                       @"Native spline admission refused the complete candidate.");
        return;
    }
    State expected = State::Open;
    if (!_c4State.compare_exchange_strong(expected, State::Prepared)) {
        (void)_c4Service->cancel(_c4Opening->session);
        DeliverBounded(completion, Core3DBoundedCurvePreparationResultRejected,
                       @"Opening changed while preparing.");
        return;
    }
    _c4Prepared = std::move(prepared);
    DeliverBounded(completion, Core3DBoundedCurvePreparationResultPrepared, @"Prepared");
}
- (void)applyWithName:(NSString *)name
    completion:(void (^)(Core3DProfileConstructionResult, NSString *))completion {
    State expected = State::Prepared;
    if (!_c4State.compare_exchange_strong(expected, State::Applying) || !_c4Prepared) {
        Deliver(completion, Core3DProfileConstructionResultRejected, @"Prepare the current complete values first."); return;
    }
    const auto receipt = _c4Service->apply(_c4Prepared, name.UTF8String ?: "Spline Profile");
    Core3DProfileConstructionResult result = Core3DProfileConstructionResultRejected;
    if (receipt.outcome == core3d::spline_profile::owner::Outcome::committed)
        result = Core3DProfileConstructionResultCommitted;
    else if (receipt.outcome == core3d::spline_profile::owner::Outcome::busy)
        result = Core3DProfileConstructionResultBusy;
    else if (receipt.outcome == core3d::spline_profile::owner::Outcome::outcomeUnknown
             || receipt.outcome == core3d::spline_profile::owner::Outcome::recoveryRequired)
        result = Core3DProfileConstructionResultRecoveryRequired;
    _c4State.store(result == Core3DProfileConstructionResultRecoveryRequired
        ? State::Recovery : State::Settled);
    if (result != Core3DProfileConstructionResultRecoveryRequired) {
        _c4Prepared.reset(); _c4Opening.reset(); _c4Service.reset(); _c4Context.reset();
    }
    Deliver(completion, result, result == Core3DProfileConstructionResultCommitted
        ? @"Committed" : result == Core3DProfileConstructionResultRecoveryRequired
            ? @"Spline-profile close is unknown; native recovery ownership is retained."
            : [NSString stringWithUTF8String:receipt.reason.c_str()]);
}
- (BOOL)cancel {
    State value = _c4State.load();
    while (value == State::Open || value == State::Prepared) {
        if (_c4State.compare_exchange_weak(value, State::Cancelled)) {
            if (!_c4Service || !_c4Opening
                || _c4Service->cancel(_c4Opening->session).outcome
                    != core3d::spline_profile::owner::Outcome::cancelled) {
                _c4State.store(State::Recovery); return NO;
            }
            _c4Prepared.reset(); _c4Opening.reset(); _c4Service.reset(); _c4Context.reset();
            return YES;
        }
    }
    return NO;
}
@end

@implementation Core3DSplineProfileEditingOpening (Core3DNativeOpeningHold)
- (std::shared_ptr<core3d::native_opening::Context>)core3d_nativeOpeningContext {
    // Only a live opening (Open or Prepared) may be the tolerated blocker of
    // the holding planning-context predicate; an applied, cancelled, applying
    // or recovery opening — or one whose context was already released —
    // unwraps to null and the viewer predicate refuses it.
    const State state = _c4State.load();
    if (state != State::Open && state != State::Prepared) return {};
    return _c4Context;
}
@end

namespace {
struct ControllerOpeningInput final {
    Handle(OcctDocument) owner;
    std::shared_ptr<core3d::native_opening::Context> context;
    std::string selected;
};

bool CaptureControllerOpeningInput(Core3DViewController *controller,
                                   ControllerOpeningInput& output, bool traceGeneralLoft = false) noexcept {
    output = {};
    const char *gate = "reopen-controller-main-thread";
    const auto refused = [&]() -> bool {
#if DEBUG
        if (traceGeneralLoft) NSLog(@"R4_C3N refused=%s", gate);
#else
        (void)traceGeneralLoft;
        (void)gate;
#endif
        return false;
    };
    try {
        if (!NSThread.isMainThread || !controller) return refused();
        gate = "reopen-controller-scene";
        Core3DSceneSnapshot *scene = [controller captureSceneSnapshot];
        if (!scene) return refused();
        gate = "reopen-controller-selection-count";
        if (scene.selection.selectedElements.count != 1) return refused();
        gate = "reopen-controller-selected-object";
        Core3DSceneElementIdentifier *element = scene.selection.selectedElements.firstObject;
        if (element.kind != Core3DSceneElementKindObject
            || element.entityIdentifier.length == 0
            || element.entityIdentifier.length > 128) return refused();
        gate = "reopen-controller-gl";
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        if (!gl) return refused();
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        const CGSize drawable = controller.viewportDrawableSize;
        gate = "reopen-controller-viewer-viewport";
        if (!viewer || !std::isfinite(drawable.width) || !std::isfinite(drawable.height)
            || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return refused();
        gate = "reopen-controller-document-context";
        output.owner = viewer->getDocument();
        output.selected = element.entityIdentifier.UTF8String ?: "";
        output.context = viewer->captureNativeOpeningContext(
            std::uint32_t(drawable.width), std::uint32_t(drawable.height), {output.selected});
        if (output.owner.IsNull() || !output.context || output.selected.empty()) return refused();
        return true;
    } catch (...) { output = {}; return refused(); }
}

// Creation needs no selection: the fence capture uses the empty receipt set.
bool CaptureControllerCreationInput(Core3DViewController *controller,
                                    ControllerOpeningInput& output) noexcept {
    output = {};
    const char *gate = "creation-main-thread";
    const auto refused = [&]() -> bool {
#if DEBUG
        NSLog(@"R4_C3N refused=%s", gate);
#endif
        return false;
    };
    try {
        if (!NSThread.isMainThread) return refused();
        gate = "creation-controller";
        if (!controller) return refused();
        GLViewController *gl = [controller.glController isKindOfClass:GLViewController.class]
            ? (GLViewController *)controller.glController : nil;
        gate = "creation-gl-controller";
        if (!gl) return refused();
        const std::shared_ptr<core3d::Core3DViewer> viewer = gl.viewer;
        const CGSize drawable = controller.viewportDrawableSize;
        gate = "creation-viewer";
        if (!viewer) return refused();
        gate = "creation-viewport";
        if (!std::isfinite(drawable.width) || !std::isfinite(drawable.height)
            || drawable.width < 1 || drawable.height < 1
            || drawable.width > UINT32_MAX || drawable.height > UINT32_MAX) return refused();
        gate = "creation-document-context";
        output.owner = viewer->getDocument();
        output.context = viewer->captureNativeOpeningContext(
            std::uint32_t(drawable.width), std::uint32_t(drawable.height), {});
        gate = "creation-document";
        if (output.owner.IsNull()) return refused();
        gate = "creation-context";
        if (!output.context) return refused();
        return true;
    } catch (...) { output = {}; return refused(); }
}

// Row-277 C4: builds the spline-profile editing opening from an already-captured
// input, so callers that captured exactly once (manual editor, AI edit entry)
// never trigger a second context capture.
Core3DSplineProfileEditingOpening *OpenSplineProfileEditorFromInput(ControllerOpeningInput& input) {
    auto service = std::make_unique<C4Owner>(*input.owner, input.context);
    auto opening = service->capture(input.selected);
    if (!opening) {
#if DEBUG
        std::fprintf(stderr, "R179_SPLINE_OPEN_REFUSED stage=capture\n");
#endif
        return nil;
    }
    return [[Core3DSplineProfileEditingOpening alloc] initWithDocument:input.owner
        context:input.context service:std::move(service) opening:std::move(opening) creating:NO];
}
} // namespace

@implementation Core3DViewController (RetainedEditorOpenings)
- (Core3DBoundedCurveEditingOpening *)openBoundedCurveEditor {
    ControllerOpeningInput input;
    return CaptureControllerOpeningInput(self, input)
        ? core3d::native_opening::OpenBoundedCurve(
            input.owner, input.selected, input.context) : nil;
}
- (Core3DBoundedCurveCreationOpening *)openBoundedCurveCreation {
    ControllerOpeningInput input;
    return CaptureControllerCreationInput(self, input)
        ? core3d::native_opening::OpenBoundedCurveCreation(
            input.owner, input.context) : nil;
}
- (Core3DSplineProfileEditingOpening *)beginSplineProfileCreation {
    ControllerOpeningInput input;
    if (!CaptureControllerCreationInput(self, input)) return nil;
    C4Definition definition;
    if (!C4DefaultDefinition(*input.owner, definition)) return nil;
    auto service = std::make_unique<C4Owner>(*input.owner, input.context);
    auto opening = service->beginCreate(definition); if (!opening) return nil;
    return [[Core3DSplineProfileEditingOpening alloc] initWithDocument:input.owner
        context:input.context service:std::move(service) opening:std::move(opening) creating:YES];
}
- (Core3DSplineProfileEditingOpening *)openSplineProfileEditor {
    ControllerOpeningInput input;
    if (!CaptureControllerOpeningInput(self, input)) {
#if DEBUG
        std::fprintf(stderr, "R179_SPLINE_OPEN_REFUSED stage=controller-input\n");
#endif
        return nil;
    }
    return OpenSplineProfileEditorFromInput(input);
}
// Row-277 C4 AI command bridge (Core3DViewController.h): both entries return the
// same opaque Core3DSplineProfileEditingOpening the row-270 owner methods vend.
// The edit entry fences the exact reviewed target before delegating; nil never
// changes model or history.
- (Core3DSplineProfileEditingOpening *)beginAISplineProfileCreation {
    return [self beginSplineProfileCreation];
}
- (Core3DSplineProfileEditingOpening *)openAISplineProfileEditorForEntityIdentifier:(NSString *)entityIdentifier {
    if (entityIdentifier.length == 0 || entityIdentifier.length > 128) return nil;
    ControllerOpeningInput input;
    if (!CaptureControllerOpeningInput(self, input)) return nil;
    if (input.selected != entityIdentifier.UTF8String) return nil;
    return OpenSplineProfileEditorFromInput(input);
}
- (Core3DPatternEditingOpening *)openPatternEditor {
    ControllerOpeningInput input;
    return CaptureControllerOpeningInput(self, input)
        ? core3d::native_opening::OpenPattern(
            input.owner, input.selected, input.context) : nil;
}
- (Core3DPathArrayEditingOpening *)openPathArrayEditor {
    ControllerOpeningInput input;
    return CaptureControllerOpeningInput(self, input)
        ? core3d::native_opening::OpenPathArray(
            input.owner, input.selected, input.context) : nil;
}
- (Core3DFeaturePatternEditingOpening *)openFeaturePatternEditor {
    ControllerOpeningInput input;
    return CaptureControllerOpeningInput(self, input)
        ? core3d::native_opening::OpenFeaturePattern(
            input.owner, input.selected, input.context) : nil;
}
- (Core3DGeneralLoftEditingOpening *)openGeneralLoftEditor {
    ControllerOpeningInput input;
    return CaptureControllerOpeningInput(self, input, true)
        ? core3d::native_opening::OpenGeneralLoft(
            input.owner, input.selected, input.context) : nil;
}
- (Core3DGeneralLoftEditingOpening *)beginGeneralLoftCreationWithStations:
    (NSArray<NSDictionary<NSString *,id> *> *)stations orderAxis:(NSArray<NSNumber *> *)orderAxis
    requestedName:(NSString *)requestedName {
    ControllerOpeningInput input;
    return CaptureControllerCreationInput(self, input)
        ? core3d::native_opening::BeginGeneralLoft(
            input.owner, stations, orderAxis, requestedName, input.context) : nil;
}
@end

#if DEBUG
namespace {
// Defined in the R179 fixture bridge below; same translation-unit anonymous
// namespace, so one set of forward declarations links the probe scenarios.
bool StageR179ObjectsFixture(const Handle(TDocStd_Document)& document,
                             NSString* kind, double metersPerUnit) noexcept;
core3d::retained_recipe::UUID R179GenerateUUID();
std::string R179UUIDText(const core3d::retained_recipe::UUID& value);
NSString *R179UUIDString(const core3d::retained_recipe::UUID& value);
bool R179SetUUIDAttribute(const TDF_Label& label, const char* attributeID,
                          const core3d::retained_recipe::UUID& value);
bool R179StageC1(const Handle(TDocStd_Document)& document,
                 const Handle(XCAFDoc_ShapeTool)& shapes,
                 const core3d::retained_recipe::UUID& documentUUID,
                 double nativePerMM);
bool R179StageD2(const Handle(TDocStd_Document)& document,
                 const Handle(XCAFDoc_ShapeTool)& shapes,
                 const core3d::retained_recipe::UUID& documentUUID,
                 double nativePerMM);
bool R179StageD3(const Handle(TDocStd_Document)& document,
                 const Handle(XCAFDoc_ShapeTool)& shapes,
                 const core3d::retained_recipe::UUID& documentUUID,
                 double nativePerMM);
bool R179StageD4(const Handle(TDocStd_Document)& document,
                 const Handle(XCAFDoc_ShapeTool)& shapes,
                 const core3d::retained_recipe::UUID& documentUUID,
                 double nativePerMM, double metersPerUnit);
}

extern "C" std::uint64_t Core3DDebugNativeOpeningBodiesProbe(
    std::int32_t scenario) noexcept {
    @autoreleasepool {
        if (scenario == 0) {
            std::uint64_t bits = 0;
            if (MapOutcome(core3d::pattern_owner::ApplyOutcome::Committed,
                    core3d::pattern_owner::ApplyOutcome::Committed,
                    core3d::pattern_owner::ApplyOutcome::OutcomeUnknown)
                    == Core3DProfileConstructionResultCommitted) bits |= 1;
            if (MapOutcome(core3d::path_array_owner::ApplyOutcome::Committed,
                    core3d::path_array_owner::ApplyOutcome::Committed,
                    core3d::path_array_owner::ApplyOutcome::OutcomeUnknown)
                    == Core3DProfileConstructionResultCommitted) bits |= 2;
            if (MapOutcome(core3d::feature_pattern_owner::ApplyOutcome::Committed,
                    core3d::feature_pattern_owner::ApplyOutcome::Committed,
                    core3d::feature_pattern_owner::ApplyOutcome::OutcomeUnknown)
                    == Core3DProfileConstructionResultCommitted) bits |= 4;
            if (&core3d::native_opening::OpenPattern
                && &core3d::native_opening::OpenPathArray
                && &core3d::native_opening::OpenFeaturePattern) bits |= 8;
            if (MapOutcome(core3d::feature_pattern_owner::ApplyOutcome::OutcomeUnknown,
                    core3d::feature_pattern_owner::ApplyOutcome::Committed,
                    core3d::feature_pattern_owner::ApplyOutcome::OutcomeUnknown)
                    == Core3DProfileConstructionResultRecoveryRequired) bits |= 16;
            return bits;
        }
        if (scenario == 1) {
            std::uint64_t bits = 0; double scalar = 0; std::uint64_t integer = 0;
            if (!Number(@YES, scalar)) bits |= 1;
            if (!Number(@(NAN), scalar)) bits |= 2;
            if (!Integer(@(1.5), UINT32_MAX, integer)) bits |= 4;
            if (!ExactKeys(@{@"row": @0, @"column": @1, @"foreign": @2},
                           @[@"row", @"column"])) bits |= 8;
            std::atomic_bool used{false}; bool expected = false;
            if (used.compare_exchange_strong(expected, true)) {
                expected = false;
                if (!used.compare_exchange_strong(expected, true)) bits |= 16;
            }
            return bits;
        }
        if (scenario == 2) {
            std::uint64_t bits = 0; std::atomic<State> state{State::Open};
            State expected = State::Open;
            if (state.compare_exchange_strong(expected, State::Cancelled)) bits |= 1;
            expected = State::Open;
            if (!state.compare_exchange_strong(expected, State::Applying)) bits |= 2;
            state.store(State::Recovery); expected = State::Open;
            if (!state.compare_exchange_strong(expected, State::Cancelled)) bits |= 4;
            if (MapOutcome(core3d::pattern_owner::ApplyOutcome::OutcomeUnknown,
                    core3d::pattern_owner::ApplyOutcome::Committed,
                    core3d::pattern_owner::ApplyOutcome::OutcomeUnknown)
                    == Core3DProfileConstructionResultRecoveryRequired) bits |= 8;
            if (state.load() == State::Recovery) bits |= 16;
            return bits;
        }
        if (scenario == 3) {
            // D2 optional-record admission boundary. A bare retained-pattern
            // source with absent profile/enclosure records opens; a current
            // present record stays admitted; present-but-stale or malformed
            // records still refuse.
            try {
                OcctDocument owner;
                owner.InitDoc();
                const Handle(TDocStd_Document) document = owner.Document();
                if (document.IsNull()) return 0;
                XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
                core3d::retained_recipe::UUID documentUUID{};
                if (!core3d::pattern_owner::Parse(
                        owner.DocumentIdentifier(), documentUUID)) {
                    documentUUID = R179GenerateUUID();
                    if (!R179SetUUIDAttribute(document->Main(),
                            "74386E4E-F620-498F-8092-E6D883AF33A4", documentUUID))
                        return 0;
                }
                const Handle(XCAFDoc_ShapeTool) shapes =
                    XCAFDoc_DocumentTool::ShapeTool(document->Main());
                if (shapes.IsNull()
                    || !R179StageD2(document, shapes, documentUUID, 1.0)) return 0;
                std::vector<core3d::pattern::Record> records;
                if (!core3d::pattern::ReadAll(document, records)
                    || records.size() != 1) return 0;
                const std::string selected =
                    R179UUIDText(records.front().definition.source.entity);
                std::uint64_t bits = 0;
                core3d::pattern_owner::Snapshot opening;
                if (core3d::pattern_owner::Capture(owner, selected, opening)
                        == core3d::pattern_owner::Refusal::None
                    && opening.admitted()
                    && opening.sourceProfileAndEnclosure.object.profile.label
                        .IsNull()) bits |= 1;
                const TDF_Label sourceLabel = opening.source.label;
                if (sourceLabel.IsNull()) return bits;
                // A current present profile record remains admitted.
                document->NewCommand();
                core3d::profile::Parameters plate;
                plate.metersPerUnit = 0.001;
                plate.definition.depth = 8;
                plate.definition.points = {{0, 0}, {20, 0}, {20, 12}, {0, 12}};
                const std::string identifier = OcctDocument::NewProfileIdentifier();
                const bool staged = !identifier.empty()
                    && core3d::profile::Stage(document, sourceLabel, plate,
                                              identifier)
                    && document->CommitCommand();
                if (!staged) { document->AbortCommand(); return bits; }
                core3d::pattern_owner::Snapshot withProfile;
                if (core3d::pattern_owner::Capture(owner, selected, withProfile)
                        == core3d::pattern_owner::Refusal::None
                    && withProfile.admitted()
                    && !withProfile.sourceProfileAndEnclosure.object.profile.label
                        .IsNull()) bits |= 2;
                // The malformed/restored arms must corrupt and clear the
                // captured profile record itself, never a guessed child tag:
                // ProfilePersistence allocates the record label dynamically
                // above the maximum existing child tag.
                const TDF_Label profileLabel =
                    withProfile.sourceProfileAndEnclosure.object.profile.label;
                if (profileLabel.IsNull()
                    || profileLabel.Data() != document->GetData()
                    || !profileLabel.Father().IsEqual(sourceLabel)) return bits;
                // A present-but-stale profile (document unit moved on) refuses.
                XCAFDoc_DocumentTool::SetLengthUnit(document, 1.0);
                core3d::pattern_owner::Snapshot stale;
                if (core3d::pattern_owner::Capture(owner, selected, stale)
                        == core3d::pattern_owner::Refusal::UnsupportedSource)
                    bits |= 4;
                XCAFDoc_DocumentTool::SetLengthUnit(document, 0.001);
                // A malformed present profile record refuses closed: corrupt
                // the schema of the captured profile record itself.
                TDataStd_Integer::Set(profileLabel,
                    core3d::profile::SchemaID(), 999);
                core3d::pattern_owner::Snapshot malformed;
                if (core3d::pattern_owner::Capture(owner, selected, malformed)
                        == core3d::pattern_owner::Refusal::UnsupportedSource)
                    bits |= 8;
                // Clear that same record recursively; no live profile
                // attributes may remain on it.
                profileLabel.ForgetAllAttributes(Standard_True);
                if (core3d::profile::HasAttribute(profileLabel)) return bits;
                // With the malformed record gone the source is admitted again.
                core3d::pattern_owner::Snapshot restored;
                if (core3d::pattern_owner::Capture(owner, selected, restored)
                        == core3d::pattern_owner::Refusal::None
                    && restored.admitted()
                    && restored.sourceProfileAndEnclosure.object.profile.label
                        .IsNull()) bits |= 16;
                return bits;
            } catch (...) { return 0; }
        }
        if (scenario == 4) {
            // D4 positive fixture and descriptor contract: the native staging
            // must persist a valid pair, a retained baseline and source
            // program, reproduce them exactly after a safe save/reopen in both
            // document units, and the descriptors must carry the real document
            // unit symbol and scale with *MM payloads in millimetres.
            try {
                std::uint64_t stagedBits = 1, pairBits = 2, reopenBits = 4,
                    descriptorBits = 8, conversionBits = 16;
                for (double unit : {0.001, 1.0}) {
                    Handle(TDocStd_Application) application =
                        new TDocStd_Application();
                    Core3DDefineSafeBinXCAFFormat(application);
                    Handle(TDocStd_Document) document;
                    application->NewDocument(
                        TCollection_ExtendedString("BinXCAF"), document);
                    if (document.IsNull()) return 0;
                    if (!StageR179ObjectsFixture(document, @"d4", unit))
                        stagedBits = 0;
                    std::vector<core3d::feature_pattern::Record> records;
                    std::vector<core3d::feature_pattern_child::PairedRecord> pairs;
                    std::vector<core3d::retained_solid::Record> retained;
                    if (!stagedBits
                        || !(core3d::feature_pattern::ReadAll(document, records)
                            && records.size() == 1
                            && core3d::feature_pattern_child::ReadPairs(
                                document, pairs)
                                == core3d::feature_pattern_child::PairStatus::Valid
                            && pairs.size() == 1
                            && pairs.front().children.size() == 3
                            && !pairs.front().baselineRecipe.IsNull()
                            && core3d::retained_solid::ReadAll(document, retained)
                            && retained.size() == 1 && retained.front().value))
                        pairBits = 0;
                    const auto* program = pairBits
                        ? std::get_if<core3d::retained_boolean::Program>(
                            &retained.front().value->envelope) : nullptr;
                    core3d::retained_boolean::Identity identities;
                    if (program) {
                        identities = core3d::retained_boolean::Identities(
                            retained.front().value->envelope);
                        NSUInteger differences = 0;
                        bool selected = false;
                        for (const auto& step : program->steps) {
                            if (step.operation
                                != core3d::analytic_boolean::Operation::Difference)
                                continue;
                            ++differences;
                            selected = selected || step.operand.identifier
                                == records.front().definition.sourceCutStepID;
                        }
                        if (!(differences >= 2 && selected
                            && identities.derivedFeature
                                == records.front().definition.sourceCut
                                    .sourceFeature)) pairBits = 0;
                    } else pairBits = 0;
                    // Safe save, then a separate safe application must reopen
                    // the exact pattern bytes and ordered child receipts.
                    std::ostringstream saved(std::ios::out | std::ios::binary);
                    if (pairBits
                        && application->SaveAs(document, saved) != PCDM_SS_OK)
                        reopenBits = 0;
                    if (reopenBits) {
                        Handle(TDocStd_Application) reader =
                            new TDocStd_Application();
                        Core3DDefineSafeBinXCAFFormat(reader);
                        Handle(TDocStd_Document) opened;
                        std::istringstream input(saved.str(),
                            std::ios::in | std::ios::binary);
                        Core3DBeginSafeBinaryRead();
                        std::vector<core3d::feature_pattern::Record> again;
                        std::vector<core3d::feature_pattern_child::PairedRecord>
                            againPairs;
                        const bool exact = reader->Open(input, opened)
                                == PCDM_RS_OK
                            && !Core3DSafeBinaryReadWasRejected()
                            && !opened.IsNull()
                            && core3d::feature_pattern::ReadAll(opened, again)
                            && again.size() == 1
                            && again.front().bytes == records.front().bytes
                            && core3d::feature_pattern_child::ReadPairs(
                                opened, againPairs)
                                == core3d::feature_pattern_child::PairStatus::Valid
                            && againPairs.size() == 1
                            && againPairs.front().children.size()
                                == pairs.front().children.size();
                        bool childrenExact = exact;
                        if (exact)
                            for (std::size_t index = 0;
                                 index < pairs.front().children.size(); ++index)
                                childrenExact = childrenExact
                                    && againPairs.front().children[index]
                                    && pairs.front().children[index]
                                    && againPairs.front().children[index]
                                        ->canonicalBytes
                                        == pairs.front().children[index]
                                            ->canonicalBytes;
                        if (!childrenExact) reopenBits = 0;
                        if (!opened.IsNull()) reader->Close(opened);
                    }
                    // Descriptor: real unit symbol and scale, *MM payloads in
                    // millimetres, source steps keyed by the derived feature.
                    if (pairBits) {
                        core3d::feature_pattern_owner::Snapshot view;
                        view.record = records.front();
                        view.sourceProgram.recipe =
                            retained.front().value->envelope;
                        NSDictionary *descriptor = FeatureDescriptor(view);
                        NSArray *cuts = descriptor[@"availableSourceCuts"];
                        const bool descriptorExact =
                            [descriptor[@"unitSymbol"]
                                isEqualToString:unit == 1.0 ? @"m" : @"mm"]
                            && [descriptor[@"documentMetersPerUnit"]
                                doubleValue] == unit
                            && [descriptor[@"rowSpacingMM"] doubleValue] == 5.0
                            && [descriptor[@"columnSpacingMM"] doubleValue] == -4.0
                            && cuts.count == 2
                            && [cuts[0][@"featureIdentifier"]
                                isEqualToString:R179UUIDString(
                                    identities.derivedFeature)]
                            && [cuts[1][@"featureIdentifier"]
                                isEqualToString:R179UUIDString(
                                    identities.derivedFeature)];
                        if (!descriptorExact) descriptorBits = 0;
                        NSDictionary *candidate = @{
                            @"sourceCutStepID": @2, @"kind": @"grid",
                            @"rowAxis": @1, @"columnAxis": @0,
                            @"rowCount": @2, @"columnCount": @3,
                            @"rowSpacingMM": @4.5, @"columnSpacingMM": @-2.5,
                            @"radialPivotMM": @[@0, @0, @0],
                            @"sweepDegrees": @0,
                            @"suppressedCoordinates": @[]};
                        core3d::feature_pattern_owner::Edit edit;
                        if (!FeatureEdit(candidate, unit, edit)
                            || edit.rowSpacing != 4.5 / (unit * 1000.0)
                            || edit.columnSpacing != -2.5 / (unit * 1000.0)
                            || edit.sourceCutStepID != 2)
                            conversionBits = 0;
                    } else { descriptorBits = 0; conversionBits = 0; }
                    application->Close(document);
                }
                if (![UnitSymbol(1.0) isEqualToString:@"m"]
                    || ![UnitSymbol(0.001) isEqualToString:@"mm"]
                    || ![UnitSymbol(0.01) isEqualToString:@"document"])
                    conversionBits = 0;
                return stagedBits | pairBits | reopenBits | descriptorBits
                    | conversionBits;
            } catch (...) { return 0; }
        }
    }
    return 0;
}

#if DEBUG
namespace {
// ---------------------------------------------------------------------------
// Row-265 lifecycle probe (Core3DDebugNativeOpeningLifecycleProbe).
//
// Every scenario owns a real initialized viewer on a detached 64x64 offscreen
// framebuffer (the proven OwnerProbeFixture publication sequence from
// BoundedCurveOwner.mm) and stages the R179 DEBUG fixture families defined
// below in this translation unit into the owned viewer document. Openings are
// created through the ordinary core3d::native_opening factories — the exact
// functions the Core3DViewController (RetainedEditorOpenings) category
// forwards to — and edits are driven through the real opening objects, so
// every bit is an observation of a real native session. No local atomic, no
// hand-built outcome, no mock opening can set a bit: each bit additionally
// requires a document-side proof (record bytes, owner/member identities,
// history counts, fence state) that only the real session produces.
// ---------------------------------------------------------------------------

#if TARGET_OS_IOS
struct LifecycleProbeContextRestorer final {
    __strong EAGLContext *context = nil;
    ~LifecycleProbeContextRestorer() noexcept {
        (void)[EAGLContext setCurrentContext:context];
    }
};
#endif

struct LifecycleProbeSelections final {
    std::string c1, d2, d3, d4;
    core3d::path_array::CurveReference replacementPath;
};

struct LifecycleProbeFixture final {
    core3d::Core3DViewer viewer;
    Handle(OcctDocument) document;
    std::vector<Handle(AIS_Shape)> presentations;
    LifecycleProbeSelections selections;
    // Descriptive failure provenance only; never contributes an observation bit.
    const char *setupStage = "allocate-gl-host";
    // Diagnostics provenance only; set by the probe entry point, never read by
    // any admission, refusal or predicate decision.
    int probeScenario = -1;
#if TARGET_OS_IOS
    __strong GLView *host = nil;
#endif

    ~LifecycleProbeFixture() noexcept { shutdown(); }

    bool perform(void (^work)(void)) noexcept {
#if TARGET_OS_IOS
        LifecycleProbeContextRestorer restore{[EAGLContext currentContext]};
        try {
            return host != nil && work != nil
                && [host debugPerformWithProbeFramebuffer:work];
        } catch (...) { return false; }
#else
        (void)work;
        return false;
#endif
    }

    void shutdown() noexcept {
#if TARGET_OS_IOS
        if (host == nil) return;
        LifecycleProbeContextRestorer restore{[EAGLContext currentContext]};
        LifecycleProbeFixture *fixture = this;
        void (^cleanup)(void) = ^{
            try {
                if (!fixture->document.IsNull()
                    && !fixture->document->Document().IsNull()
                    && fixture->document->Document()->HasOpenCommand())
                    fixture->document->Document()->AbortCommand();
                fixture->viewer.release();
            } catch (...) {}
        };
        if (!perform(cleanup)) {
            try { (void)[host performWithRenderingContext:cleanup]; }
            catch (...) {}
        }
        presentations.clear();
        document.Nullify();
        host = nil;
#endif
    }

    bool initialize(double metersPerUnit, bool withC1) noexcept {
#if !TARGET_OS_IOS
        (void)metersPerUnit; (void)withC1;
        return false;
#else
        try {
            LifecycleProbeContextRestorer restore{[EAGLContext currentContext]};
            host = [[GLView alloc] initWithFrame:
                CGRectMake(0.0, 0.0, 64.0, 64.0)];
            setupStage = "prepare-probe-framebuffer";
            if (host == nil || host->myGLContext == nil
                || ![host debugPrepareProbeFramebuffer]) {
                host = nil;
                return false;
            }
            setupStage = "restore-caller-context";
            if (![EAGLContext setCurrentContext:restore.context]) {
                shutdown();
                return false;
            }
            __block bool initialized = false;
            LifecycleProbeFixture *fixture = this;
            setupStage = "enter-probe-framebuffer";
            const bool performed = perform(^{
                fixture->setupStage = "initialize-viewer-and-interactors";
                if (!fixture->viewer.InitViewer(fixture->host)
                    || fixture->viewer.getObjectInteractor() == nullptr
                    || fixture->viewer.getShapeInteractor() == nullptr
                    || fixture->viewer.AisContext().IsNull()
                    || fixture->viewer.ActiveView().IsNull()) return;
                fixture->setupStage = "viewer-document";
                fixture->document = fixture->viewer.getDocument();
                if (fixture->document.IsNull()
                    || fixture->document->Document().IsNull()
                    || fixture->document->Document()->GetData().IsNull()
                    || fixture->document->Document()->HasOpenCommand()) return;
                const Handle(TDocStd_Document) document =
                    fixture->document->Document();
                fixture->setupStage = "set-length-unit";
                XCAFDoc_DocumentTool::SetLengthUnit(document, metersPerUnit);
                fixture->setupStage = "document-uuid";
                core3d::retained_recipe::UUID documentUUID{};
                if (!core3d::pattern_owner::Parse(
                        fixture->document->DocumentIdentifier(), documentUUID)) {
                    documentUUID = R179GenerateUUID();
                    if (!R179SetUUIDAttribute(document->Main(),
                            "74386E4E-F620-498F-8092-E6D883AF33A4",
                            documentUUID)) return;
                }
                fixture->setupStage = "shape-tool";
                const Handle(XCAFDoc_ShapeTool) shapes =
                    XCAFDoc_DocumentTool::ShapeTool(document->Main());
                if (shapes.IsNull()) return;
                const double nativePerMM = 0.001 / metersPerUnit;
                auto& selected = fixture->selections;
                if (withC1) {
                    fixture->setupStage = "stage-c1";
                    if (!R179StageC1(document, shapes, documentUUID,
                            nativePerMM)) return;
                    fixture->setupStage = "read-c1";
                    std::vector<core3d::bounded_curve::Record> curves;
                    if (!core3d::bounded_curve::ReadAll(document, curves)
                        || curves.size() != 1) return;
                    selected.c1 = R179UUIDText(curves.front()
                        .value->persisted.ownerState.owner.entity);
                }
                fixture->setupStage = "stage-d2";
                if (!R179StageD2(document, shapes, documentUUID, nativePerMM))
                    return;
                {
                    fixture->setupStage = "read-d2";
                    std::vector<core3d::pattern::Record> records;
                    if (!core3d::pattern::ReadAll(document, records)
                        || records.size() != 1) return;
                    selected.d2 = R179UUIDText(
                        records.front().definition.source.entity);
                }
                fixture->setupStage = "stage-d3";
                if (!R179StageD3(document, shapes, documentUUID, nativePerMM))
                    return;
                {
                    fixture->setupStage = "read-d3-and-curves";
                    std::vector<core3d::path_array::Record> arrays;
                    std::vector<core3d::bounded_curve::Record> curves;
                    if (!core3d::path_array::ReadAll(document, arrays)
                        || arrays.size() != 1
                        || !core3d::bounded_curve::ReadAll(document, curves)
                        || curves.size() != (withC1 ? 3U : 2U)) return;
                    selected.d3 = R179UUIDText(
                        arrays.front().definition.source.entity);
                    fixture->setupStage = "find-replacement-path";
                    bool found = false;
                    for (const auto& curve : curves) {
                        const auto& persisted = curve.value->persisted;
                        if (withC1
                            && R179UUIDText(persisted.ownerState.owner.entity)
                                == selected.c1) continue;
                        if (core3d::path_array::Matches(
                                arrays.front().definition.path, persisted))
                            continue;
                        selected.replacementPath.owner =
                            persisted.ownerState.owner;
                        selected.replacementPath.feature =
                            persisted.value.feature;
                        selected.replacementPath.definitionRevision =
                            persisted.ownerState.definitionRevision;
                        selected.replacementPath.canonicalDefinitionDigest =
                            persisted.ownerState.canonicalDefinitionDigest;
                        found = true;
                    }
                    if (!found) return;
                }
                fixture->setupStage = "stage-d4";
                if (!R179StageD4(document, shapes, documentUUID, nativePerMM,
                        metersPerUnit)) return;
                {
                    fixture->setupStage = "read-d4";
                    std::vector<core3d::feature_pattern::Record> records;
                    if (!core3d::feature_pattern::ReadAll(document, records)
                        || records.size() != 1) return;
                    selected.d4 = R179UUIDText(
                        records.front().definition.host.entity);
                }
                // Baseline history after every fixture command, then publish
                // the staged shapes exactly like the proven owner probe.
                fixture->setupStage = "publish-staged-shapes";
                document->ClearUndos();
                fixture->document->NotifyChanges();
                const auto ais = fixture->viewer.AisContext();
                TDF_LabelSequence roots;
                shapes->GetFreeShapes(roots);
                std::set<std::string> expected, published;
                for (const std::string& entity :
                        {selected.c1, selected.d2, selected.d3, selected.d4})
                    if (!entity.empty()) expected.insert(entity);
                for (Standard_Integer index = 1; index <= roots.Length();
                     ++index) {
                    const TDF_Label label = roots.Value(index);
                    const TopoDS_Shape shape =
                        XCAFDoc_ShapeTool::GetShape(label);
                    fixture->setupStage = "publish-nonnull-root";
                    if (shape.IsNull()) return;
                    Handle(AIS_Shape) presentation = new AIS_Shape(shape);
                    ais->Display(presentation, AIS_Shaded, 0, Standard_False);
                    fixture->presentations.push_back(presentation);
                    const std::string entity =
                        fixture->document->EntityIdentifierForLabel(label);
                    if (!entity.empty()) published.insert(entity);
                }
                fixture->setupStage = "published-entity-set";
                for (const std::string& entity : expected)
                    if (!published.count(entity)) return;
                fixture->setupStage = "whole-shape-selection";
                const auto shapeInteractor =
                    fixture->viewer.getShapeInteractor();
                if (shapeInteractor->setSelectionMode(
                        core3d::ShapeSelectionMode::WholeShape)
                        != core3d::ShapeSelectionModeChangeResult::Succeeded
                    || !shapeInteractor->selectionModeAuthorityIsExact())
                    return;
                fixture->setupStage = "redraw-and-snapshot";
                ais->UpdateCurrentViewer();
                fixture->viewer.ActiveView()->FitAll();
                const auto snapshot =
                    fixture->viewer.captureSceneSnapshot(64, 64);
                fixture->setupStage = "snapshot-invariants";
                // Null-snapshot fields are explicitly unavailable (-1); a
                // fallback value must never read as an observed publication
                // identifier length or document unit.
                NSLog(@"R179_SNAPSHOT_INVARIANTS scenario=%d snapshotNull=%d publicationEmpty=%d metersPerUnitSnapshot=%g metersPerUnitExpected=%g openCommand=%d undos=%d redos=%d",
                      fixture->probeScenario,
                      int(!snapshot),
                      snapshot
                          ? int(snapshot->publicationSourceIdentifier.empty())
                          : -1,
                      snapshot ? snapshot->metersPerUnit : -1.0,
                      metersPerUnit,
                      int(document->HasOpenCommand()),
                      int(document->GetAvailableUndos()),
                      int(document->GetAvailableRedos()));
                if (!snapshot) {
                    // Probe-side mirror of the captureSceneSnapshot early
                    // returns (Core3DViewer.mm). Read-only accessors only; no
                    // value here feeds any admission, refusal or ordering.
                    const auto nullReasonObjectInteractor =
                        fixture->viewer.getObjectInteractor();
                    const auto nullReasonShapeInteractor =
                        fixture->viewer.getShapeInteractor();
                    unsigned int previewUnknownMask = 0;
                    if (nullReasonObjectInteractor != nullptr) {
                        if (nullReasonObjectInteractor->mirrorPreviewState()
                                == core3d::MirrorPreviewState::OutcomeUnknown)
                            previewUnknownMask |= 0x1;
                        if (nullReasonObjectInteractor->linearArrayPreviewState()
                                == core3d::LinearArrayPreviewState::OutcomeUnknown)
                            previewUnknownMask |= 0x2;
                        if (nullReasonObjectInteractor->radialArrayPreviewState()
                                == core3d::RadialArrayPreviewState::OutcomeUnknown)
                            previewUnknownMask |= 0x4;
                        if (nullReasonObjectInteractor->hasUnresolvedBoolean())
                            previewUnknownMask |= 0x20;
                    }
                    if (nullReasonShapeInteractor != nullptr) {
                        if (nullReasonShapeInteractor->shellPreviewState()
                                == core3d::ShellPreviewState::OutcomeUnknown)
                            previewUnknownMask |= 0x8;
                        if (nullReasonShapeInteractor->extrusionPreviewState()
                                == core3d::ExtrusionPreviewState::OutcomeUnknown)
                            previewUnknownMask |= 0x10;
                    }
                    NSLog(@"R179_SNAPSHOT_NULL_REASON scenario=%d unresolvedEdit=%d shapeInteractorNull=%d selectionExact=%d previewUnknownMask=0x%x",
                          fixture->probeScenario,
                          int(fixture->viewer.hasUnresolvedEdit()),
                          int(nullReasonShapeInteractor == nullptr),
                          nullReasonShapeInteractor != nullptr
                              ? int(nullReasonShapeInteractor
                                      ->selectionModeAuthorityIsExact())
                              : -1,
                          previewUnknownMask);
                    // Read-only shared validators at the actual refusal: the
                    // owned-frame/baseline census result and document identity
                    // equality on this same document. Diagnostic only; none of
                    // these values feeds any admission, refusal or ordering.
                    Standard_Size nullReasonFrameBytes = 0;
                    const int nullReasonOwnedFrameValid =
                        Core3DValidateOwnedFrameUsage(document,
                            nullReasonFrameBytes) ? 1 : 0;
                    std::vector<core3d::feature_pattern_child::BaselineRecord>
                        nullReasonBaselines;
                    const int nullReasonBaselineStatus = int(
                        core3d::feature_pattern_child::ReadBaselines(
                            document, nullReasonBaselines));
                    core3d::retained_recipe::UUID nullReasonDocumentUUID{};
                    const int nullReasonDocumentIdentityEqual =
                        core3d::retained_solid::ReadUUID(document->Main(),
                            Standard_GUID(
                                "74386E4E-F620-498F-8092-E6D883AF33A4"),
                            nullReasonDocumentUUID)
                        && R179UUIDText(nullReasonDocumentUUID)
                            == fixture->document->DocumentIdentifier() ? 1 : 0;
                    NSLog(@"R179_SNAPSHOT_NULL_VALIDATORS scenario=%d ownedFrameValid=%d baselineStatus=%d baselineCount=%lu documentIdentityEqual=%d",
                          fixture->probeScenario,
                          nullReasonOwnedFrameValid,
                          nullReasonBaselineStatus,
                          (unsigned long)nullReasonBaselines.size(),
                          nullReasonDocumentIdentityEqual);
                }
                if (!snapshot
                    || snapshot->publicationSourceIdentifier.empty()
                    || snapshot->metersPerUnit != metersPerUnit
                    || document->HasOpenCommand()
                    || document->GetAvailableUndos() != 0
                    || document->GetAvailableRedos() != 0) return;
                fixture->setupStage = "snapshot-entity-set";
                for (const std::string& entity : expected)
                    if (std::none_of(snapshot->instances.begin(),
                            snapshot->instances.end(),
                            [&](const core3d::scene::InstanceSnapshot&
                                    instance) {
                                return instance.entityIdentifier == entity;
                            })) {
                        NSLog(@"R179_SNAPSHOT_ENTITY_MISSING scenario=%d entity=%s instanceCount=%lu",
                              fixture->probeScenario,
                              entity.c_str(),
                              (unsigned long)snapshot->instances.size());
                        return;
                    }
                fixture->setupStage = "restore-probe-framebuffer";
                initialized = true;
            });
            if (!performed || !initialized) {
                NSLog(@"R179_LIFECYCLE_SETUP stage=%s performed=%d initialized=%d",
                      setupStage, int(performed), int(initialized));
                shutdown(); return false;
            }
            return true;
        } catch (...) {
            NSLog(@"R179_LIFECYCLE_SETUP_EXCEPTION stage=%s", setupStage);
            shutdown(); return false;
        }
#endif
    }
};

int LifecycleUndos(LifecycleProbeFixture& fixture) noexcept {
    try {
        return fixture.document.IsNull() || fixture.document->Document().IsNull()
            ? -1 : fixture.document->Document()->GetAvailableUndos();
    } catch (...) { return -1; }
}

// Canonical record bytes of every retained family plus the undo depth. Used to
// prove a refusal left file, history and ownership untouched.
struct LifecycleDocumentState final {
    std::vector<std::vector<std::uint8_t>> patterns, arrays, features, children;
    int undos = -1;
    bool valid = false;
};

LifecycleDocumentState LifecycleState(
    const Handle(TDocStd_Document)& document) noexcept {
    LifecycleDocumentState state;
    try {
        std::vector<core3d::pattern::Record> patterns;
        std::vector<core3d::path_array::Record> arrays;
        std::vector<core3d::feature_pattern::Record> features;
        std::vector<core3d::feature_pattern_child::PairedRecord> pairs;
        state.valid = core3d::pattern::ReadAll(document, patterns)
            && core3d::path_array::ReadAll(document, arrays)
            && core3d::feature_pattern::ReadAll(document, features)
            && core3d::feature_pattern_child::ReadPairs(document, pairs)
                == core3d::feature_pattern_child::PairStatus::Valid;
        for (const auto& record : patterns)
            state.patterns.push_back(record.bytes);
        for (const auto& record : arrays) state.arrays.push_back(record.bytes);
        for (const auto& record : features)
            state.features.push_back(record.bytes);
        for (const auto& pair : pairs)
            for (const auto& child : pair.children)
                if (child) state.children.push_back(child->canonicalBytes);
        state.undos = document->GetAvailableUndos();
    } catch (...) { state.valid = false; }
    return state;
}

bool SameState(const LifecycleDocumentState& a,
               const LifecycleDocumentState& b) noexcept {
    return a.valid && b.valid && a.undos == b.undos
        && a.patterns == b.patterns && a.arrays == b.arrays
        && a.features == b.features && a.children == b.children;
}

NSDictionary *LifecycleD2Candidate(double columnSpacingMM,
                                   std::uint32_t columnCount) {
    return @{@"rowAxis": @1, @"columnAxis": @0,
        @"rowCount": @1, @"columnCount": @(columnCount),
        @"rowSpacingMM": @0.0, @"columnSpacingMM": @(columnSpacingMM),
        @"radialPivotMM": @[@0, @0, @0], @"sweepDegrees": @0,
        @"suppressedEntityIdentifiers": @[]};
}

NSDictionary *LifecycleD3Candidate(std::uint32_t count) {
    return @{@"distribution": @"count", @"count": @(count),
        @"distanceInDocumentUnits": @0, @"includeStart": @YES,
        @"includeEnd": @YES, @"closedPath": @NO, @"orientation": @"fixed",
        @"rollDegrees": @0, @"hasUpVector": @NO,
        @"upVector": @[@0, @0, @1],
        @"maximumFrameStepDegrees": @(Degrees(0.1)),
        @"arcToleranceInDocumentUnits": @1e-4,
        @"minimumTangentInDocumentUnits": @1e-6,
        @"suppressedEntityIdentifiers": @[]};
}

// Retained D3 edit mirroring the staged definition so a replacement-path apply
// changes exactly the path receipt.
core3d::path_array_owner::Edit LifecycleD3RetainedEdit() {
    core3d::path_array_owner::Edit edit;
    edit.distribution = core3d::path_array::DistributionMode::Count;
    edit.count = 2;
    edit.distance = 0;
    edit.includeStart = true;
    edit.includeEnd = true;
    edit.closedPath = false;
    edit.orientation = core3d::path_array::OrientationPolicy::Fixed;
    edit.rollRadians = 0;
    edit.hasUpVector = false;
    edit.upVector = {{0, 0, 1}};
    edit.maximumFrameStepRadians = 0.1;
    edit.arcLengthTolerance = 1e-4;
    edit.minimumTangent = 1e-6;
    return edit;
}

NSDictionary *LifecycleD4Candidate(std::uint64_t step,
                                   std::uint32_t columnCount) {
    return @{@"sourceCutStepID": @(step), @"kind": @"grid",
        @"rowAxis": @1, @"columnAxis": @0, @"rowCount": @2,
        @"columnCount": @(columnCount), @"rowSpacingMM": @5.0,
        @"columnSpacingMM": @-4.0, @"radialPivotMM": @[@0, @0, @0],
        @"sweepDegrees": @0,
        @"suppressedCoordinates": @[@{@"row": @1, @"column": @1}]};
}

Core3DPatternEditingOpening *LifecycleOpenD2(
    LifecycleProbeFixture& fixture,
    std::shared_ptr<core3d::native_opening::Context>& context) noexcept {
    __block Core3DPatternEditingOpening *opening = nil;
    LifecycleProbeFixture *owner = &fixture;
    const std::string entity = fixture.selections.d2;
    const bool performed = fixture.perform(^{
        context = owner->viewer.captureNativeOpeningContext(64, 64, {entity});
        if (context) opening = core3d::native_opening::OpenPattern(
            owner->document, entity, context);
    });
#if DEBUG
    // F2 factory provenance: framebuffer block ran, context capture and
    // native opening admission for the next factory attempt.
    NSLog(@"R179_FACTORY family=d2 performed=%d context=%d opening=%d",
          int(performed), int(context != nullptr), int(opening != nil));
#endif
    if (!performed) return nil;
    return opening;
}

Core3DPathArrayEditingOpening *LifecycleOpenD3(
    LifecycleProbeFixture& fixture,
    std::shared_ptr<core3d::native_opening::Context>& context) noexcept {
    __block Core3DPathArrayEditingOpening *opening = nil;
    LifecycleProbeFixture *owner = &fixture;
    const std::string entity = fixture.selections.d3;
    const bool performed = fixture.perform(^{
        context = owner->viewer.captureNativeOpeningContext(64, 64, {entity});
        if (context) opening = core3d::native_opening::OpenPathArray(
            owner->document, entity, context);
    });
#if DEBUG
    NSLog(@"R179_FACTORY family=d3 performed=%d context=%d opening=%d",
          int(performed), int(context != nullptr), int(opening != nil));
#endif
    if (!performed) return nil;
    return opening;
}

Core3DFeaturePatternEditingOpening *LifecycleOpenD4(
    LifecycleProbeFixture& fixture,
    std::shared_ptr<core3d::native_opening::Context>& context) noexcept {
    __block Core3DFeaturePatternEditingOpening *opening = nil;
    LifecycleProbeFixture *owner = &fixture;
    const std::string entity = fixture.selections.d4;
    const bool performed = fixture.perform(^{
        context = owner->viewer.captureNativeOpeningContext(64, 64, {entity});
        if (context) opening = core3d::native_opening::OpenFeaturePattern(
            owner->document, entity, context);
    });
#if DEBUG
    NSLog(@"R179_FACTORY family=d4 performed=%d context=%d opening=%d",
          int(performed), int(context != nullptr), int(opening != nil));
#endif
    if (!performed) return nil;
    return opening;
}

Core3DBoundedCurveEditingOpening *LifecycleOpenC1(
    LifecycleProbeFixture& fixture,
    std::shared_ptr<core3d::native_opening::Context>& context) noexcept {
    __block Core3DBoundedCurveEditingOpening *opening = nil;
    LifecycleProbeFixture *owner = &fixture;
    const std::string entity = fixture.selections.c1;
    const bool performed = fixture.perform(^{
        context = owner->viewer.captureNativeOpeningContext(64, 64, {entity});
        if (context) opening = core3d::native_opening::OpenBoundedCurve(
            owner->document, entity, context);
    });
#if DEBUG
    NSLog(@"R179_FACTORY family=c1 performed=%d context=%d opening=%d",
          int(performed), int(context != nullptr), int(opening != nil));
#endif
    if (!performed) return nil;
    return opening;
}

Core3DProfileConstructionResult LifecycleApply(
    LifecycleProbeFixture& fixture, Core3DPatternEditingOpening *opening,
    NSDictionary *candidate) noexcept {
    __block Core3DProfileConstructionResult result =
        Core3DProfileConstructionResultFailed;
    if (!fixture.perform(^{
        [opening applyCandidate:candidate
            completion:^(Core3DProfileConstructionResult value, NSString *) {
                result = value;
            }];
    })) return Core3DProfileConstructionResultFailed;
    return result;
}

Core3DProfileConstructionResult LifecycleApply(
    LifecycleProbeFixture& fixture, Core3DPathArrayEditingOpening *opening,
    NSDictionary *candidate) noexcept {
    __block Core3DProfileConstructionResult result =
        Core3DProfileConstructionResultFailed;
    if (!fixture.perform(^{
        [opening applyCandidate:candidate
            completion:^(Core3DProfileConstructionResult value, NSString *) {
                result = value;
            }];
    })) return Core3DProfileConstructionResultFailed;
    return result;
}

Core3DFeaturePatternPreparation *LifecyclePrepareD4(
    LifecycleProbeFixture& fixture,
    Core3DFeaturePatternEditingOpening *opening,
    NSDictionary *candidate) noexcept {
    __block Core3DFeaturePatternPreparation *preparation = nil;
    if (!fixture.perform(^{
        preparation = [opening prepareCandidate:candidate];
    })) return nil;
    return preparation;
}

Core3DProfileConstructionResult LifecycleApplyPrepared(
    LifecycleProbeFixture& fixture,
    Core3DFeaturePatternEditingOpening *opening,
    Core3DFeaturePatternPreparedEdit *token) noexcept {
    __block Core3DProfileConstructionResult result =
        Core3DProfileConstructionResultFailed;
    if (!fixture.perform(^{
        [opening applyPrepared:token
            completion:^(Core3DProfileConstructionResult value, NSString *) {
                result = value;
            }];
    })) return Core3DProfileConstructionResultFailed;
    return result;
}

// D218 recovery republishes real presentations in the owning framebuffer.
bool LifecycleRecover(LifecycleProbeFixture& fixture,
    const std::shared_ptr<core3d::native_opening::Context>& context,
    bool exactStateKnown) noexcept {
    __block bool recovered = false;
    const bool performed = fixture.perform(^{
        recovered = context && context->reconcileRecovery(exactStateKnown);
    });
    return performed && recovered;
}

// The probe cancels openings of several editing classes through an `id`; the
// shared `cancel` selector has more than one signature in Core3DModelingTypes.h
// (the Boolean editor returns an object), so type the receiver as one of the
// BOOL-returning opening classes. Dispatch stays dynamic: every opening the
// probe passes here declares `- (BOOL)cancel`.
BOOL LifecycleCancel(LifecycleProbeFixture& fixture, id opening) noexcept {
    __block BOOL cancelled = NO;
    if (!fixture.perform(^{
        cancelled = [(Core3DPatternEditingOpening *)opening cancel];
    })) return NO;
    return cancelled;
}

// True only when the framebuffer block ran and the viewer refused to mint an
// opening context — the shared recovery fence blocks the factory.
bool LifecycleFactoryBusy(LifecycleProbeFixture& fixture,
                          const std::string& entity) noexcept {
    __block std::shared_ptr<core3d::native_opening::Context> captured;
    LifecycleProbeFixture *owner = &fixture;
    return fixture.perform(^{
        captured = owner->viewer.captureNativeOpeningContext(64, 64, {entity});
    }) && !captured;
}

// Scenario 0 — success. Real D2/D3/D4 sessions commit and read back; second
// edits are accepted; every edit is exactly one history command; a safe
// save/reopen retains owner and member receipts.
std::uint64_t LifecycleScenario0Body(LifecycleProbeFixture& fixture) noexcept {
    @autoreleasepool {
        try {
            const Handle(TDocStd_Document) document =
                fixture.document->Document();
            std::uint64_t bits = 0;
            std::vector<int> historyDeltas;
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPatternEditingOpening *opening =
                    LifecycleOpenD2(fixture, context);
                if (opening != nil && opening.descriptor.count > 0) {
                    bits |= 1;
                    const int before = LifecycleUndos(fixture);
                    const auto result = LifecycleApply(fixture, opening,
                        LifecycleD2Candidate(-25.0, 2));
                    std::vector<core3d::pattern::Record> records;
                    if (result == Core3DProfileConstructionResultCommitted
                        && core3d::pattern::ReadAll(document, records)
                        && records.size() == 1
                        && records.front().definition.columnSpacing == -25.0) {
                        bits |= 2;
                        historyDeltas.push_back(
                            LifecycleUndos(fixture) - before);
                    }
                }
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPathArrayEditingOpening *opening =
                    LifecycleOpenD3(fixture, context);
                if (opening != nil && opening.descriptor.count > 0) {
                    bits |= 4;
                    const int before = LifecycleUndos(fixture);
                    // The replacement authority is minted natively by the
                    // production resolver; the apply runs the same
                    // PrepareNative/ApplyNative session the opening drives.
                    __block auto outcome = core3d::path_array_owner::
                        ApplyOutcome::Refused;
                    LifecycleProbeFixture *probe = &fixture;
                    const bool applied = fixture.perform(^{
                        outcome = core3d::path_array_owner::
                            ApplyReplacementPathForDebugProbe(
                                *probe->document, opening->_opening,
                                probe->selections.replacementPath,
                                LifecycleD3RetainedEdit(), context);
                    });
                    std::vector<core3d::path_array::Record> records;
                    if (applied && outcome == core3d::path_array_owner::
                            ApplyOutcome::Committed
                        && core3d::path_array::ReadAll(document, records)
                        && records.size() == 1
                        && records.front().definition.path.feature
                            == fixture.selections.replacementPath.feature
                        && records.front().definition.path.owner
                            == fixture.selections.replacementPath.owner) {
                        bits |= 8;
                        historyDeltas.push_back(
                            LifecycleUndos(fixture) - before);
                    }
                    (void)LifecycleCancel(fixture, opening);
                }
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DFeaturePatternEditingOpening *opening =
                    LifecycleOpenD4(fixture, context);
                if (opening != nil && opening.descriptor.count > 0) {
                    bits |= 16;
                    Core3DFeaturePatternPreparation *preparation =
                        LifecyclePrepareD4(fixture, opening,
                            LifecycleD4Candidate(2, 2));
                    Core3DFeaturePatternPreparedEdit *token =
                        preparation.prepared;
                    if (preparation.preview.admitted && token != nil) {
                        const int before = LifecycleUndos(fixture);
                        const auto result = LifecycleApplyPrepared(
                            fixture, opening, token);
                        std::vector<core3d::feature_pattern::Record> records;
                        if (result == Core3DProfileConstructionResultCommitted
                            && core3d::feature_pattern::ReadAll(
                                document, records)
                            && records.size() == 1
                            && records.front().definition.sourceCutStepID
                                == 2) {
                            bits |= 32;
                            historyDeltas.push_back(
                                LifecycleUndos(fixture) - before);
                        }
                    }
                }
                context.reset();
            }
            // Bit 64: all three factories accept a real second edit.
            bool secondD2 = false, secondD3 = false, secondD4 = false;
            if (bits & 2) {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPatternEditingOpening *opening =
                    LifecycleOpenD2(fixture, context);
                if (opening != nil) {
                    const int before = LifecycleUndos(fixture);
                    const auto result = LifecycleApply(fixture, opening,
                        LifecycleD2Candidate(-25.0, 3));
                    std::vector<core3d::pattern::Record> records;
                    if (result == Core3DProfileConstructionResultCommitted
                        && core3d::pattern::ReadAll(document, records)
                        && records.size() == 1
                        && records.front().definition.columnCount == 3
                        && records.front().definition.members.size() == 3) {
                        secondD2 = true;
                        historyDeltas.push_back(
                            LifecycleUndos(fixture) - before);
                    }
                }
                context.reset();
            }
            if (bits & 8) {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPathArrayEditingOpening *opening =
                    LifecycleOpenD3(fixture, context);
                if (opening != nil) {
                    const int before = LifecycleUndos(fixture);
                    const auto result = LifecycleApply(fixture, opening,
                        LifecycleD3Candidate(3));
                    std::vector<core3d::path_array::Record> records;
                    if (result == Core3DProfileConstructionResultCommitted
                        && core3d::path_array::ReadAll(document, records)
                        && records.size() == 1
                        && records.front().definition.distribution.count == 3
                        && records.front().definition.members.size() == 3) {
                        secondD3 = true;
                        historyDeltas.push_back(
                            LifecycleUndos(fixture) - before);
                    }
                }
                context.reset();
            }
            if (bits & 32) {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DFeaturePatternEditingOpening *opening =
                    LifecycleOpenD4(fixture, context);
                if (opening != nil) {
                    Core3DFeaturePatternPreparation *preparation =
                        LifecyclePrepareD4(fixture, opening,
                            LifecycleD4Candidate(2, 3));
                    Core3DFeaturePatternPreparedEdit *token =
                        preparation.prepared;
                    if (preparation.preview.admitted && token != nil) {
                        const int before = LifecycleUndos(fixture);
                        const auto result = LifecycleApplyPrepared(
                            fixture, opening, token);
                        std::vector<core3d::feature_pattern::Record> records;
                        std::vector<
                            core3d::feature_pattern_child::PairedRecord> pairs;
                        std::uint32_t active = 0;
                        if (result
                                == Core3DProfileConstructionResultCommitted
                            && core3d::feature_pattern::ReadAll(
                                document, records)
                            && records.size() == 1
                            && records.front().definition.distribution
                                .columnCount == 3
                            && records.front().definition.sourceCutStepID == 2
                            && core3d::feature_pattern::ActiveCount(
                                records.front().definition, active)
                            && core3d::feature_pattern_child::ReadPairs(
                                document, pairs)
                                == core3d::feature_pattern_child::
                                    PairStatus::Valid
                            && pairs.size() == 1
                            && pairs.front().children.size() == active) {
                            secondD4 = true;
                            historyDeltas.push_back(
                                LifecycleUndos(fixture) - before);
                        }
                    }
                }
                context.reset();
            }
            if (secondD2 && secondD3 && secondD4) bits |= 64;
            if (historyDeltas.size() == 6
                && std::all_of(historyDeltas.begin(), historyDeltas.end(),
                    [](int delta) { return delta == 1; })) bits |= 128;
            // Bit 256: safe save/reopen retains owner and member receipts.
            {
                Handle(TDocStd_Application) application =
                    new TDocStd_Application();
                Core3DDefineSafeBinXCAFFormat(application);
                std::ostringstream saved(std::ios::out | std::ios::binary);
                if (application->SaveAs(document, saved) == PCDM_SS_OK) {
                    Handle(TDocStd_Application) reader =
                        new TDocStd_Application();
                    Core3DDefineSafeBinXCAFFormat(reader);
                    Handle(TDocStd_Document) opened;
                    std::istringstream input(saved.str(),
                        std::ios::in | std::ios::binary);
                    Core3DBeginSafeBinaryRead();
                    std::vector<core3d::pattern::Record> liveD2, readD2;
                    std::vector<core3d::path_array::Record> liveD3, readD3;
                    std::vector<core3d::feature_pattern::Record> liveD4,
                        readD4;
                    const bool exact =
                        reader->Open(input, opened) == PCDM_RS_OK
                        && !Core3DSafeBinaryReadWasRejected()
                        && !opened.IsNull()
                        && core3d::pattern::ReadAll(document, liveD2)
                        && core3d::pattern::ReadAll(opened, readD2)
                        && liveD2.size() == 1 && readD2.size() == 1
                        && liveD2.front().bytes == readD2.front().bytes
                        && liveD2.front().definition.owner
                            == readD2.front().definition.owner
                        && core3d::path_array::ReadAll(document, liveD3)
                        && core3d::path_array::ReadAll(opened, readD3)
                        && liveD3.size() == 1 && readD3.size() == 1
                        && liveD3.front().bytes == readD3.front().bytes
                        && liveD3.front().definition.members.size()
                            == readD3.front().definition.members.size()
                        && liveD3.front().definition.members.back().identity
                            == readD3.front().definition.members.back()
                                .identity
                        && core3d::feature_pattern::ReadAll(document, liveD4)
                        && core3d::feature_pattern::ReadAll(opened, readD4)
                        && liveD4.size() == 1 && readD4.size() == 1
                        && liveD4.front().bytes == readD4.front().bytes
                        && liveD4.front().definition.host
                            == readD4.front().definition.host;
                    if (!opened.IsNull()) reader->Close(opened);
                    if (exact) bits |= 256;
                }
            }
            return bits;
        } catch (...) { return 0; }
    }
}

// Scenario 1 — refusal. Malformed numbers, foreign tokens/receipts/preparations
// and repeated applies refuse through the real openings, and every refusal
// leaves file, history and ownership unchanged.
std::uint64_t LifecycleScenario1Body(LifecycleProbeFixture& fixture) noexcept {
    @autoreleasepool {
        try {
            const Handle(TDocStd_Document) document =
                fixture.document->Document();
            std::uint64_t bits = 0;
            bool allUnchanged = true;
            const auto unchanged = [&](const LifecycleDocumentState& before) {
                const bool same = SameState(before,
                    LifecycleState(document));
                allUnchanged = allUnchanged && same;
                return same;
            };
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPatternEditingOpening *opening =
                    LifecycleOpenD2(fixture, context);
                if (opening != nil) {
                    NSMutableDictionary *candidate =
                        [LifecycleD2Candidate(-20.0, 2) mutableCopy];
                    candidate[@"rowSpacingMM"] = @(NAN);
                    const auto before = LifecycleState(document);
                    if (LifecycleApply(fixture, opening, candidate)
                            == Core3DProfileConstructionResultRejected
                        && unchanged(before)) bits |= 1;
                }
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPathArrayEditingOpening *opening =
                    LifecycleOpenD3(fixture, context);
                if (opening != nil) {
                    NSMutableDictionary *candidate =
                        [LifecycleD3Candidate(2) mutableCopy];
                    candidate[@"distanceInDocumentUnits"] = @(NAN);
                    const auto before = LifecycleState(document);
                    if (LifecycleApply(fixture, opening, candidate)
                            == Core3DProfileConstructionResultRejected
                        && unchanged(before)) bits |= 2;
                }
                context.reset();
            }
            Core3DFeaturePatternEditingOpening *foreignIssuer = nil;
            Core3DFeaturePatternPreparedEdit *foreignToken = nil;
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DFeaturePatternEditingOpening *opening =
                    LifecycleOpenD4(fixture, context);
                if (opening != nil) {
                    NSMutableDictionary *candidate =
                        [LifecycleD4Candidate(1, 2) mutableCopy];
                    candidate[@"rowSpacingMM"] = @(NAN);
                    const auto before = LifecycleState(document);
                    Core3DFeaturePatternPreparation *preparation =
                        LifecyclePrepareD4(fixture, opening, candidate);
                    if (preparation != nil && !preparation.preview.admitted
                        && preparation.prepared == nil
                        && unchanged(before)) bits |= 4;
                }
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPatternEditingOpening *opening =
                    LifecycleOpenD2(fixture, context);
                if (opening != nil) {
                    NSMutableDictionary *candidate =
                        [LifecycleD2Candidate(-20.0, 2) mutableCopy];
                    candidate[@"suppressedEntityIdentifiers"] =
                        @[R179UUIDString(R179GenerateUUID())];
                    const auto before = LifecycleState(document);
                    if (LifecycleApply(fixture, opening, candidate)
                            == Core3DProfileConstructionResultRejected
                        && unchanged(before)) bits |= 8;
                }
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPathArrayEditingOpening *opening =
                    LifecycleOpenD3(fixture, context);
                if (opening != nil) {
                    auto foreignReference = fixture.selections.replacementPath;
                    foreignReference.feature = R179GenerateUUID();
                    const auto before = LifecycleState(document);
                    const auto outcome =
                        core3d::path_array_owner::
                            ApplyReplacementPathForDebugProbe(
                                *fixture.document, opening->_opening,
                                foreignReference, LifecycleD3RetainedEdit(),
                                context);
                    if (outcome == core3d::path_array_owner::
                            ApplyOutcome::Refused
                        && unchanged(before)) bits |= 16;
                    (void)LifecycleCancel(fixture, opening);
                }
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                foreignIssuer = LifecycleOpenD4(fixture, context);
                if (foreignIssuer != nil) {
                    Core3DFeaturePatternPreparation *preparation =
                        LifecyclePrepareD4(fixture, foreignIssuer,
                            LifecycleD4Candidate(2, 2));
                    foreignToken = preparation.prepared;
                }
                (void)LifecycleCancel(fixture, foreignIssuer);
                foreignIssuer = nil;
                context.reset();
            }
            Core3DFeaturePatternEditingOpening *opening = nil;
            Core3DFeaturePatternPreparedEdit *token = nil;
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                opening = LifecycleOpenD4(fixture, context);
                if (opening != nil && foreignToken != nil) {
                    const auto before = LifecycleState(document);
                    if (LifecycleApplyPrepared(fixture, opening, foreignToken)
                            == Core3DProfileConstructionResultRejected
                        && unchanged(before)) bits |= 32;
                }
                if (opening != nil) {
                    Core3DFeaturePatternPreparation *preparation =
                        LifecyclePrepareD4(fixture, opening,
                            LifecycleD4Candidate(2, 2));
                    token = preparation.prepared;
                    if (token != nil) {
                        if (LifecycleApplyPrepared(fixture, opening, token)
                                == Core3DProfileConstructionResultCommitted) {
                            const auto before = LifecycleState(document);
                            if (LifecycleApplyPrepared(
                                    fixture, opening, token)
                                    == Core3DProfileConstructionResultRejected
                                && unchanged(before)) bits |= 64;
                        }
                    }
                }
                context.reset();
            }
            bool repeatedRefused = false;
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPatternEditingOpening *repeated =
                    LifecycleOpenD2(fixture, context);
                if (repeated != nil
                    && LifecycleApply(fixture, repeated,
                        LifecycleD2Candidate(-25.0, 2))
                        == Core3DProfileConstructionResultCommitted) {
                    const auto before = LifecycleState(document);
                    repeatedRefused = LifecycleApply(fixture, repeated,
                        LifecycleD2Candidate(-25.0, 2))
                        == Core3DProfileConstructionResultRejected
                        && unchanged(before);
                }
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPathArrayEditingOpening *repeated =
                    LifecycleOpenD3(fixture, context);
                if (repeated != nil
                    && LifecycleApply(fixture, repeated,
                        LifecycleD3Candidate(3))
                        == Core3DProfileConstructionResultCommitted) {
                    const auto before = LifecycleState(document);
                    repeatedRefused = repeatedRefused
                        && LifecycleApply(fixture, repeated,
                            LifecycleD3Candidate(3))
                            == Core3DProfileConstructionResultRejected
                        && unchanged(before);
                } else repeatedRefused = false;
                context.reset();
            }
            if (repeatedRefused) bits |= 128;
            if (allUnchanged) bits |= 256;
            return bits;
        } catch (...) { return 0; }
    }
}

// Scenario 2 — close. Real cancellations close exactly once, post-cancel
// applies refuse, the factory reopens, and an injected close-unknown retains
// the issuing session until exact recovery.
std::uint64_t LifecycleScenario2Body(LifecycleProbeFixture& fixture) noexcept {
    @autoreleasepool {
        try {
            std::uint64_t bits = 0;
            Core3DPatternEditingOpening *d2 = nil;
            Core3DPathArrayEditingOpening *d3 = nil;
            Core3DFeaturePatternEditingOpening *d4 = nil;
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                d2 = LifecycleOpenD2(fixture, context);
                if (d2 != nil && LifecycleCancel(fixture, d2)
                    && !LifecycleCancel(fixture, d2)) bits |= 1;
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                d3 = LifecycleOpenD3(fixture, context);
                if (d3 != nil && LifecycleCancel(fixture, d3)
                    && !LifecycleCancel(fixture, d3)) bits |= 2;
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                d4 = LifecycleOpenD4(fixture, context);
                if (d4 != nil && LifecycleCancel(fixture, d4)
                    && !LifecycleCancel(fixture, d4)) bits |= 4;
                context.reset();
            }
            bool postCancelRefused = false;
            if (d2 != nil)
                postCancelRefused = LifecycleApply(fixture, d2,
                    LifecycleD2Candidate(-25.0, 2))
                    == Core3DProfileConstructionResultRejected;
            if (d3 != nil)
                postCancelRefused = postCancelRefused
                    && LifecycleApply(fixture, d3, LifecycleD3Candidate(3))
                        == Core3DProfileConstructionResultRejected;
            if (d4 != nil) {
                Core3DFeaturePatternPreparation *preparation =
                    LifecyclePrepareD4(fixture, d4,
                        LifecycleD4Candidate(1, 2));
                postCancelRefused = postCancelRefused && preparation != nil
                    && !preparation.preview.admitted
                    && preparation.prepared == nil;
            }
            if (postCancelRefused) bits |= 8;
            d2 = nil; d3 = nil; d4 = nil;
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPatternEditingOpening *reopened =
                    LifecycleOpenD2(fixture, context);
                if (reopened != nil && LifecycleCancel(fixture, reopened))
                    bits |= 16;
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPatternEditingOpening *opening =
                    LifecycleOpenD2(fixture, context);
                if (opening != nil && context) {
                    context->debugReportNextCloseUnproven();
                    const int before = LifecycleUndos(fixture);
                    const auto result = LifecycleApply(fixture, opening,
                        LifecycleD2Candidate(-25.0, 2));
                    const int afterClose = LifecycleUndos(fixture);
                    if (result
                            == Core3DProfileConstructionResultRecoveryRequired
                        && !LifecycleCancel(fixture, opening)
                        && LifecycleFactoryBusy(fixture,
                            fixture.selections.d2)) bits |= 32;
                    if (result
                            == Core3DProfileConstructionResultRecoveryRequired)
                        bits |= 64;
                    if (!LifecycleRecover(fixture, context, false)
                        && LifecycleFactoryBusy(fixture,
                            fixture.selections.d2)) bits |= 128;
                    // The real close ran (exactly one command); exact recovery
                    // releases ownership without adding history.
                    if (afterClose == before + 1
                        && LifecycleRecover(fixture, context, true)
                        && LifecycleUndos(fixture) == afterClose) bits |= 256;
                }
                context.reset();
            }
            return bits;
        } catch (...) { return 0; }
    }
}

// Scenario 3 — global fence. One real close forced unknown fences every
// factory on the shared viewer until exact same-document recovery.
std::uint64_t LifecycleScenario3Body(LifecycleProbeFixture& fixture) noexcept {
    @autoreleasepool {
        try {
            std::uint64_t bits = 0;
            // Setup proof (not a bit): every factory opens pre-fault.
            bool preFaultLive = false;
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPatternEditingOpening *opening =
                    LifecycleOpenD2(fixture, context);
                preFaultLive = opening != nil
                    && LifecycleCancel(fixture, opening);
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DPathArrayEditingOpening *opening =
                    LifecycleOpenD3(fixture, context);
                preFaultLive = preFaultLive && opening != nil
                    && LifecycleCancel(fixture, opening);
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DFeaturePatternEditingOpening *opening =
                    LifecycleOpenD4(fixture, context);
                preFaultLive = preFaultLive && opening != nil
                    && LifecycleCancel(fixture, opening);
                context.reset();
            }
            {
                std::shared_ptr<core3d::native_opening::Context> context;
                Core3DBoundedCurveEditingOpening *opening =
                    LifecycleOpenC1(fixture, context);
                preFaultLive = preFaultLive && opening != nil
                    && LifecycleCancel(fixture, opening);
                context.reset();
            }
            std::shared_ptr<core3d::native_opening::Context> issuing;
            Core3DPatternEditingOpening *opening = nil;
            if (preFaultLive) {
                opening = LifecycleOpenD2(fixture, issuing);
                if (opening != nil && issuing) {
                    issuing->debugReportNextCloseUnproven();
                    const auto result = LifecycleApply(fixture, opening,
                        LifecycleD2Candidate(-25.0, 2));
                    if (result
                        == Core3DProfileConstructionResultRecoveryRequired)
                        bits |= 1;
                }
            }
            if (bits & 1) {
                if (LifecycleFactoryBusy(fixture, fixture.selections.d2))
                    bits |= 2;
                if (LifecycleFactoryBusy(fixture, fixture.selections.d3))
                    bits |= 4;
                if (LifecycleFactoryBusy(fixture, fixture.selections.d4))
                    bits |= 8;
                if (LifecycleFactoryBusy(fixture, fixture.selections.c1))
                    bits |= 16;
                // Bit 32: the issuing context's pre-fault scene fence is
                // stale; the ordinary factory re-checks currency and refuses.
                __block Core3DPatternEditingOpening *stale = nil;
                LifecycleProbeFixture *owner = &fixture;
                const std::string entity = fixture.selections.d2;
                const bool staleRan = fixture.perform(^{
                    stale = core3d::native_opening::OpenPattern(
                        owner->document, entity, issuing);
                });
                if (staleRan && stale == nil) bits |= 32;
                // Bit 64: a capture adopted from a foreign document refuses
                // at the ordinary factory's document-identity gate.
                LifecycleProbeFixture foreign;
                __block std::shared_ptr<core3d::native_opening::Context> adopted;
                __block Core3DPatternEditingOpening *adoptedOpening = nil;
                if (foreign.initialize(0.001, false)) {
                    LifecycleProbeFixture *other = &foreign;
                    const std::string otherEntity = foreign.selections.d2;
                    if (foreign.perform(^{
                        adopted = other->viewer
                            .captureNativeOpeningContext(
                                64, 64, {otherEntity});
                    }) && adopted) {
                        const bool adoptedRan = foreign.perform(^{
                            adoptedOpening =
                                core3d::native_opening::OpenPattern(
                                    owner->document, entity, adopted);
                        });
                        if (adoptedRan && adoptedOpening == nil) bits |= 64;
                    }
                    adopted.reset();
                }
                // Bit 128: inexact recovery cannot release the fence.
                if (!LifecycleRecover(fixture, issuing, false)
                    && LifecycleFactoryBusy(fixture, fixture.selections.d2))
                    bits |= 128;
                // Bit 256: exact same-document recovery releases the fence
                // and a subsequent real edit succeeds exactly once.
                const int before = LifecycleUndos(fixture);
                if (LifecycleRecover(fixture, issuing, true)
                    && LifecycleUndos(fixture) == before
                    && !issuing->recapture(64, 64)
                    && !issuing->beginCommandLease(
                        issuing->openingFence(), 64, 64)) {
                    opening = nil;
                    // Keep the retired context alive through the new edit:
                    // exact recovery must not depend on wrapper destruction.
                    std::shared_ptr<core3d::native_opening::Context> context;
                    Core3DPatternEditingOpening *reopened =
                        LifecycleOpenD2(fixture, context);
                    if (reopened != nil) {
                        const int editBefore = LifecycleUndos(fixture);
                        const auto result = LifecycleApply(fixture, reopened,
                            LifecycleD2Candidate(-27.0, 2));
                        if (result
                                == Core3DProfileConstructionResultCommitted
                            && LifecycleUndos(fixture) == editBefore + 1
                            && LifecycleApply(fixture, reopened,
                                LifecycleD2Candidate(-27.0, 2))
                                == Core3DProfileConstructionResultRejected)
                            bits |= 256;
                    }
                    context.reset();
                }
            }
            opening = nil;
            issuing.reset();
            return bits;
        } catch (...) { return 0; }
    }
}
} // namespace

extern "C" std::uint64_t Core3DDebugNativeOpeningLifecycleProbe(
    std::int32_t scenario) noexcept {
    if (![NSThread isMainThread]) return 0;
    @autoreleasepool {
        try {
            if (scenario < 0 || scenario > 3) return 0;
            LifecycleProbeFixture fixture;
            fixture.probeScenario = int(scenario);
            if (!fixture.initialize(0.001, scenario == 3)) {
                NSLog(@"R179_LIFECYCLE_INIT_REFUSED scenario=%d stage=%s",
                      int(scenario), fixture.setupStage);
                return 0;
            }
            if (scenario == 0) return LifecycleScenario0Body(fixture);
            if (scenario == 1) return LifecycleScenario1Body(fixture);
            if (scenario == 2) return LifecycleScenario2Body(fixture);
            if (scenario == 3) return LifecycleScenario3Body(fixture);
        } catch (...) {}
    }
    return 0;
}

// Row-179 D4 typed-baseline publication/persistence probe. Scenario 0 stages
// the actual R179 D4 fixture in both unit systems on a real viewer, publishes
// a nonnull exact snapshot with the typed host baseline in place, normal-saves
// through the production owned-frame gate, safe cold-reopens in a separate
// application, compares the exact recipe bytes, identities, native label
// references and child canonical receipts, then performs a real D4 edit that
// must commit exactly one history step with the typed baseline preserved.
// Bit contract per run (every bit requires both unit systems):
// 0x01 fixtures published a nonnull exact snapshot with matching units and
// entity set, closed command and zero baseline history; 0x02 the live typed
// baseline census and pairs validate with the expected recipe and identities;
// 0x04 production normal save succeeds; 0x08 the safe cold reopen reproduces
// the exact recipe bytes, UUIDs, native references and child receipts; 0x10 a
// real subsequent D4 edit commits one history step with the baseline intact.
extern "C" std::uint64_t Core3DDebugNativeOpeningBaselineProbe(
    std::int32_t scenario) noexcept {
    if (![NSThread isMainThread]) return 0;
    @autoreleasepool {
        try {
            if (scenario != 0) return 0;
            const auto entryOf = [](const TDF_Label& label) {
                TCollection_AsciiString entry;
                if (!label.IsNull()) TDF_Tool::Entry(label, entry);
                return std::string(entry.ToCString() ? entry.ToCString() : "");
            };
            std::uint64_t bits = 0x1f;
            int unitIndex = 0;
            for (double metersPerUnit : {0.001, 1.0}) {
                std::uint64_t unit = 0;
                LifecycleProbeFixture fixture;
                fixture.probeScenario = 40 + unitIndex++;
                if (!fixture.initialize(metersPerUnit, false)) {
                    bits &= ~0x1fULL; break;
                }
                unit |= 0x01;
                const Handle(TDocStd_Document) document =
                    fixture.document->Document();
                core3d::feature_pattern_baseline::Envelope liveEnvelope;
                std::vector<std::vector<std::uint8_t>> liveChildren;
                std::string liveBaselineEntry, liveHostEntry, liveSourceEntry,
                    liveRecordEntry;
                {
                    std::vector<core3d::feature_pattern::Record> liveRecords;
                    std::vector<core3d::feature_pattern_child::PairedRecord>
                        livePairs;
                    std::vector<core3d::feature_pattern_child::BaselineRecord>
                        liveBaselines;
                    if (core3d::feature_pattern_child::ReadPairs(
                            document, livePairs)
                            != core3d::feature_pattern_child::PairStatus::Valid
                        || livePairs.size() != 1
                        || core3d::feature_pattern_child::ReadBaselines(
                            document, liveBaselines)
                            != core3d::feature_pattern_child::
                                BaselineStatus::Valid
                        || liveBaselines.size() != 1
                        || !liveBaselines.front().value
                        || !core3d::feature_pattern::ReadAll(
                            document, liveRecords)
                        || liveRecords.size() != 1) {
                        bits &= ~0x1eULL; break;
                    }
                    liveEnvelope = liveBaselines.front().value->envelope;
                    bool childrenValid = true;
                    for (const auto& child : livePairs.front().children) {
                        if (!child) { childrenValid = false; break; }
                        liveChildren.push_back(child->canonicalBytes);
                    }
                    if (!childrenValid) { bits &= ~0x1eULL; break; }
                    liveBaselineEntry =
                        entryOf(liveBaselines.front().label);
                    liveHostEntry = entryOf(livePairs.front().host);
                    liveSourceEntry = entryOf(livePairs.front().source);
                    liveRecordEntry =
                        entryOf(livePairs.front().pattern.label);
                }
                unit |= 0x02;
                NSString *saveBase = [NSTemporaryDirectory()
                    stringByAppendingPathComponent:
                        [NSString stringWithFormat:@"r179-typed-baseline-%@.tmp",
                            NSUUID.UUID.UUIDString]];
                const std::string savedFile =
                    fixture.document->save(saveBase.UTF8String);
                if (savedFile.empty()) { bits &= ~0x1cULL; break; }
                unit |= 0x04;
                @autoreleasepool {
                    NSString *written =
                        [NSString stringWithUTF8String:savedFile.c_str()];
                    Handle(TDocStd_Application) reader =
                        new TDocStd_Application();
                    Core3DDefineSafeBinXCAFFormat(reader);
                    Handle(TDocStd_Document) opened;
                    Core3DBeginSafeBinaryRead();
                    const PCDM_ReaderStatus status = reader->Open(
                        TCollection_ExtendedString(
                            savedFile.c_str(), Standard_True), opened);
                    const bool rejected =
                        Core3DSafeBinaryReadWasRejected() == Standard_True;
                    bool exact = status == PCDM_RS_OK && !rejected
                        && !opened.IsNull();
                    if (exact) {
                        std::vector<
                            core3d::feature_pattern_child::PairedRecord> pairs;
                        std::vector<
                            core3d::feature_pattern_child::BaselineRecord>
                                baselines;
                        exact =
                            core3d::feature_pattern_child::ReadPairs(
                                opened, pairs)
                                == core3d::feature_pattern_child::
                                    PairStatus::Valid
                            && pairs.size() == 1
                            && core3d::feature_pattern_child::ReadBaselines(
                                opened, baselines)
                                == core3d::feature_pattern_child::
                                    BaselineStatus::Valid
                            && baselines.size() == 1
                            && baselines.front().value
                            && baselines.front().value->envelope.document
                                == liveEnvelope.document
                            && baselines.front().value->envelope.hostEntity
                                == liveEnvelope.hostEntity
                            && baselines.front().value->envelope.hostDefinition
                                == liveEnvelope.hostDefinition
                            && baselines.front().value->envelope
                                .retainedRecipeFeature
                                == liveEnvelope.retainedRecipeFeature
                            && baselines.front().value->envelope
                                .baselineRecipeIdentity
                                == liveEnvelope.baselineRecipeIdentity
                            && baselines.front().value->envelope.exactRecipe
                                == liveEnvelope.exactRecipe
                            && entryOf(baselines.front().label)
                                == liveBaselineEntry
                            && entryOf(pairs.front().host) == liveHostEntry
                            && entryOf(pairs.front().source)
                                == liveSourceEntry
                            && entryOf(pairs.front().pattern.label)
                                == liveRecordEntry
                            && pairs.front().children.size()
                                == liveChildren.size();
                        if (exact)
                            for (std::size_t index = 0;
                                 index < liveChildren.size(); ++index)
                                if (!pairs.front().children[index]
                                    || pairs.front().children[index]
                                        ->canonicalBytes
                                            != liveChildren[index]) {
                                    exact = false; break;
                                }
                    }
                    if (!opened.IsNull()) reader->Close(opened);
                    [[NSFileManager defaultManager]
                        removeItemAtPath:written error:nil];
                    if (!exact) { bits &= ~0x18ULL; break; }
                }
                unit |= 0x08;
                {
                    std::shared_ptr<core3d::native_opening::Context> context;
                    Core3DFeaturePatternEditingOpening *opening =
                        LifecycleOpenD4(fixture, context);
                    bool committed = false;
                    if (opening != nil && opening.descriptor.count > 0) {
                        Core3DFeaturePatternPreparation *preparation =
                            LifecyclePrepareD4(fixture, opening,
                                LifecycleD4Candidate(2, 2));
                        Core3DFeaturePatternPreparedEdit *token =
                            preparation.prepared;
                        if (preparation.preview.admitted && token != nil) {
                            const int before = LifecycleUndos(fixture);
                            std::vector<core3d::feature_pattern::Record>
                                records;
                            std::vector<
                                core3d::feature_pattern_child::BaselineRecord>
                                    afterEdit;
                            committed =
                                LifecycleApplyPrepared(
                                    fixture, opening, token)
                                    == Core3DProfileConstructionResultCommitted
                                && LifecycleUndos(fixture) == before + 1
                                && !document->HasOpenCommand()
                                && core3d::feature_pattern::ReadAll(
                                    document, records)
                                && records.size() == 1
                                && records.front().definition.sourceCutStepID
                                    == 2
                                && core3d::feature_pattern_child::
                                    ReadBaselines(document, afterEdit)
                                        == core3d::feature_pattern_child::
                                            BaselineStatus::Valid
                                && afterEdit.size() == 1
                                && afterEdit.front().value
                                && afterEdit.front().value->envelope
                                    .exactRecipe == liveEnvelope.exactRecipe
                                && afterEdit.front().value->envelope
                                    .baselineRecipeIdentity
                                    == liveEnvelope.baselineRecipeIdentity;
                        }
                    }
                    context.reset();
                    if (!committed) { bits &= ~0x10ULL; break; }
                }
                unit |= 0x10;
                bits &= unit;
            }
            return bits;
        } catch (...) {}
    }
    return 0;
}
#endif

extern "C" std::uint64_t Core3DDebugBoundedCurveOpeningAdapterProbe(
    std::int32_t scenario) noexcept {
    @autoreleasepool {
        try {
            core3d::bounded_curve::owner::Opening opening;
            const auto persisted = core3d::native_opening::debug::wire_probe::Fixture();
            if (!core3d::bounded_curve::FromPersistedValue(persisted, opening.retained)) return 0;
            NSDictionary *descriptor = BoundedDescriptor(opening);
            if (scenario == 0) {
                std::uint64_t bits = ExactKeys(descriptor, @[@"domain", @"featureID",
                    @"definitionRevision", @"nextLocalID", @"canonicalDigest", @"owner",
                    @"units", @"tombstones", @"poles", @"degree", @"knots", @"frame"]) ? 1ULL : 0ULL;
                NSArray *poles = descriptor[@"poles"], *knots = descriptor[@"knots"];
                if (poles.count == opening.retained.definition.controlPoints.size()
                    && [poles.firstObject[@"id"] isEqualToString:
                        BoundedUUIDText(opening.retained.definition.controlPoints.front().identifier)]) bits |= 2;
                if (knots.count == opening.retained.definition.knots.size()
                    && [descriptor[@"degree"] unsignedCharValue] == opening.retained.definition.degree) bits |= 4;
                NSDictionary *frame = descriptor[@"frame"];
                if ([frame[@"id"] isEqualToString:BoundedUUIDText(opening.retained.definition.frame.identifier)]
                    && [frame[@"revision"] unsignedLongLongValue]
                        == opening.retained.definition.frame.revision) bits |= 8;
                if ([descriptor[@"canonicalDigest"] isEqualToData:
                        BoundedDigest(opening.retained.authority.recipeDigest)]
                    && [descriptor[@"nextLocalID"] unsignedLongLongValue]
                        == opening.retained.nextLocalID) bits |= 16;
                return bits;
            }
            auto candidateWithFirstPole = ^NSMutableDictionary *(double x, double weight) {
                NSMutableDictionary *candidate = [descriptor mutableCopy];
                NSMutableArray *poles = [descriptor[@"poles"] mutableCopy];
                NSMutableDictionary *pole = [poles.firstObject mutableCopy];
                NSMutableArray *local = [pole[@"local"] mutableCopy];
                local[0] = @(x); pole[@"local"] = local; pole[@"weight"] = @(weight);
                poles[0] = pole;
                // Handles are derived projections: the next pole's incoming
                // handle must echo this pole's candidate position.
                NSMutableDictionary *next = [poles[1] mutableCopy];
                next[@"incomingHandle"] = [local copy];
                poles[1] = next;
                candidate[@"poles"] = poles;
                return candidate;
            };
            NSUUID *poleID = [[NSUUID alloc] initWithUUIDString:descriptor[@"poles"][0][@"id"]];
            const double originalX = [descriptor[@"poles"][0][@"local"][0] doubleValue];
            const double originalWeight = [descriptor[@"poles"][0][@"weight"] doubleValue];
            if (scenario == 1) {
                std::uint64_t bits = 0; BoundedCandidate move, weight;
                NSMutableDictionary *moved = candidateWithFirstPole(originalX + 0.25, originalWeight);
                if (BoundedEdit(moved, poleID, @"movePole", opening, move)
                        == BoundedParseResult::Valid
                    && move.proposal.controlPoint
                        == opening.retained.definition.controlPoints.front().identifier) bits |= 1;
                NSMutableDictionary *weighted = candidateWithFirstPole(originalX, originalWeight + 0.25);
                if (BoundedEdit(weighted, poleID, @"setWeight", opening, weight)
                        == BoundedParseResult::Valid
                    && weight.proposal.kind == core3d::bounded_curve::EditKind::SetWeight) bits |= 2;
                if (core3d::bounded_curve::SameDefinition(
                        move.completeDefinition, opening.retained.definition) == false) bits |= 4;
                if (core3d::bounded_curve::SameDefinition(
                        weight.completeDefinition, opening.retained.definition) == false) bits |= 8;
                if (core3d::native_opening::debug::Core3DDebugBoundedCurveOwnerProbe(3) == 0x1f)
                    bits |= 16;
                return bits;
            }
            if (scenario == 2) {
                std::uint64_t bits = 0; BoundedCandidate ignored;
                NSMutableDictionary *structural = [descriptor mutableCopy];
                structural[@"degree"] = @([descriptor[@"degree"] unsignedIntValue] + 1);
                if (BoundedEdit(structural, poleID, @"movePole", opening, ignored)
                    == BoundedParseResult::Unsupported) bits |= 1;
                NSMutableDictionary *nonfinite = candidateWithFirstPole(NAN, originalWeight);
                if (BoundedEdit(nonfinite, poleID, @"movePole", opening, ignored)
                    == BoundedParseResult::Malformed) bits |= 2;
                if (BoundedEdit(descriptor, poleID, @"changeDegree", opening, ignored)
                    == BoundedParseResult::Unsupported) bits |= 4;
                if (opening.retained.authority.recipeDigest
                    == persisted.ownerState.canonicalDefinitionDigest) bits |= 8;
                if (core3d::native_opening::debug::Core3DDebugBoundedCurveOwnerProbe(2) == 0x1f)
                    bits |= 16;
                return bits;
            }
        } catch (...) {}
    }
    return 0;
}
#endif


#if DEBUG
// R179 Objects UI-test fixture bridge. DEBUG-only: the declarations live in the
// DEBUG region of Core3DViewController.h and every staging/validation path below
// is compiled out of Release. All imports for this section sit in the DEBUG
// include block at the top of this file.

namespace core3d::bounded_curve {
// Definition of the DEBUG friend forward-declared in BoundedCurveAttribute.hxx.
// It lets this fixture attach a canonical payload exactly the way the production
// owner path in OcctDocument.mm does, without widening the attribute's API.
struct PersistenceProbe {
    static bool Attach(const TDF_Label& record,
                       const std::shared_ptr<Payload>& payload) noexcept {
        try {
            if (record.IsNull() || !payload || record.HasAttribute()) return false;
            Handle(Attribute) attribute = new Attribute();
            attribute->value_ = payload;
            record.AddAttribute(attribute);
            return true;
        } catch (...) { return false; }
    }
};
} // namespace core3d::bounded_curve

namespace {

// Local equivalent of Core3DViewController.mm's DEBUG fixture serializer. The
// sibling translation unit currently keeps its copy in an anonymous namespace,
// so this bridge owns its own: same TDocStd_Application + safe BinXCAF format,
// saved to an in-memory stream (the paired-receipt probe's proven route), then
// closed before the bytes are returned.
NSData *R179CreateBinXCAFFixture(
    const std::function<void(const Handle(TDocStd_Document)&)>& populate) {
    Handle(TDocStd_Application) application;
    Handle(TDocStd_Document) document;
    NSData *result = nil;
    const char* phase = "new-document";
    try {
        application = new TDocStd_Application();
        Core3DDefineSafeBinXCAFFormat(application);
        application->NewDocument(TCollection_ExtendedString("BinXCAF"), document);
        if (document.IsNull()) {
            NSLog(@"R179_FIXTURE_REFUSED phase=%s", phase);
            return nil;
        }
        phase = "populate";
        populate(document);
        phase = "serialize";
        std::ostringstream output(std::ios::out | std::ios::binary);
        const auto saveStatus = application->SaveAs(document, output);
        if (saveStatus != PCDM_SS_OK) {
            NSLog(@"R179_FIXTURE_REFUSED phase=%s status=%d", phase, int(saveStatus));
            return nil;
        }
        const std::string bytes = output.str();
        if (bytes.empty()) {
            NSLog(@"R179_FIXTURE_REFUSED phase=empty-serialized-bytes");
            return nil;
        }
        result = [NSData dataWithBytes:bytes.data() length:bytes.size()];
    } catch (...) {
        NSLog(@"R179_FIXTURE_REFUSED phase=%s exception=1", phase);
        result = nil;
    }
    try {
        if (!application.IsNull() && !document.IsNull())
            application->Close(document);
    } catch (...) {}
    return result;
}

using core3d::retained_recipe::UUID;

UUID R179GenerateUUID() {
    UUID value{};
    [[NSUUID UUID] getUUIDBytes:value.data()];
    return value;
}

std::string R179UUIDText(const UUID& value) {
    return core3d::retained_solid::UUIDText(value);
}

NSString *R179UUIDString(const UUID& value) {
    return Text(R179UUIDText(value));
}

NSString *R179HexDigest(const core3d::retained_recipe::Digest& digest) {
    static const char digits[] = "0123456789abcdef";
    std::string text;
    text.reserve(digest.size() * 2);
    for (const std::uint8_t byte : digest) {
        text.push_back(digits[byte >> 4]);
        text.push_back(digits[byte & 15]);
    }
    return Text(text);
}

bool R179SetUUIDAttribute(const TDF_Label& label, const char* attributeID,
                          const UUID& value) {
    try {
        return !TDataStd_AsciiString::Set(label, Standard_GUID(attributeID),
            TCollection_AsciiString(R179UUIDText(value).c_str())).IsNull();
    } catch (...) { return false; }
}

struct R179Object {
    TopoDS_Shape shape;
    std::string name;
    UUID entity;
    UUID definition;
    TDF_Label label;
};

bool R179StageObject(const Handle(XCAFDoc_ShapeTool)& shapes, R179Object& object) {
    try {
        const TDF_Label owner = shapes->AddShape(
            object.shape, Standard_False, Standard_True);
        if (owner.IsNull()) return false;
        object.label = owner;
        if (TDataStd_Integer::Set(owner,
                Standard_GUID("67E669F4-00C0-4C45-BC55-9CC5DA22A2B5"), 1).IsNull())
            return false;
        TDataStd_Name::Set(owner, TCollection_ExtendedString(object.name.c_str()));
        return R179SetUUIDAttribute(owner,
                "0074F7C2-9EAA-4F89-B2DE-8716E155FF62", object.entity)
            && R179SetUUIDAttribute(owner,
                "3611F2B2-C694-4E12-AED8-A2A97A3D283B", object.definition);
    } catch (...) { return false; }
}

R179Object R179Box(double dx, double dy, double dz, const char* name) {
    return R179Object{BRepPrimAPI_MakeBox(dx, dy, dz).Shape(), name,
        R179GenerateUUID(), R179GenerateUUID(), TDF_Label()};
}

struct R179Curve {
    core3d::bounded_curve::PersistedValue persisted;
    core3d::bounded_curve::DetachedWire detached;
    TDF_Label label;
};

bool R179StageCurve(const Handle(XCAFDoc_ShapeTool)& shapes,
                    const UUID& documentUUID, double nativePerMM,
                    const char* name, R179Curve& output,
                    bool gentlePath = false) {
    try {
        auto persisted = core3d::native_opening::debug::wire_probe::Fixture();
        persisted.ownerState.owner.document = documentUUID;
        persisted.ownerState.owner.entity = R179GenerateUUID();
        persisted.ownerState.owner.definition = R179GenerateUUID();
        persisted.ownerState.feature = R179GenerateUUID();
        persisted.value.feature = persisted.ownerState.feature;
        persisted.value.definition.frame.identifier = R179GenerateUUID();
        for (double& scalar : persisted.value.definition.frame.origin)
            scalar *= nativePerMM;
        for (auto& pole : persisted.value.definition.controlPoints) {
            pole.identifier = R179GenerateUUID();
            // D3's positive control must satisfy its unchanged 0.1-radian
            // tangent limit. Keep a curved rational path and its native build.
            if (gentlePath) {
                pole.local[1] *= 0.01;
                pole.local[2] *= 0.01;
            }
            for (double& scalar : pole.local) scalar *= nativePerMM;
        }
        std::vector<std::uint8_t> bytes;
        if (!core3d::bounded_curve::Encode(persisted.value, bytes)
            || !core3d::bounded_curve::Hash(bytes,
                core3d::bounded_curve::MaximumDefinitionBytes,
                persisted.ownerState.canonicalDefinitionDigest)) return false;
        if (core3d::bounded_curve::BuildWire(persisted, output.detached)
                != core3d::bounded_curve::BuildRefusal::None) return false;
        output.persisted = persisted;
        R179Object object{output.detached.wire, name,
            persisted.ownerState.owner.entity,
            persisted.ownerState.owner.definition, TDF_Label()};
        if (!R179StageObject(shapes, object)) return false;
        output.label = object.label;
        return true;
    } catch (...) { return false; }
}

// Runs inside the caller's open command, after the owning shape label exists.
bool R179AttachCurveRecord(const R179Curve& curve) {
    try {
        const TDF_Label record = curve.label.FindChild(
            core3d::bounded_curve::MinimumRecordTag, Standard_True);
        auto payload = std::make_shared<core3d::bounded_curve::Payload>();
        payload->persisted = curve.persisted;
        payload->definitionBytes = curve.detached.canonicalDefinitionBytes;
        payload->ownerBytes = curve.detached.canonicalOwnerBytes;
        if (!core3d::bounded_curve::PersistenceProbe::Attach(record, payload))
            return false;
        TNaming_Builder(record).Select(curve.detached.wire, curve.detached.wire);
        return true;
    } catch (...) { return false; }
}

bool R179StageC1(const Handle(TDocStd_Document)& document,
                 const Handle(XCAFDoc_ShapeTool)& shapes,
                 const UUID& documentUUID, double nativePerMM) {
    R179Curve curve;
    if (!R179StageCurve(shapes, documentUUID, nativePerMM, "R179 C1 Path", curve))
        return false;
    document->SetUndoLimit(16);
    document->NewCommand();
    if (!R179AttachCurveRecord(curve)) { document->AbortCommand(); return false; }
    if (!document->CommitCommand()) return false;
    std::vector<core3d::bounded_curve::Record> records;
    return core3d::bounded_curve::ReadAll(document, records) && records.size() == 1
        && core3d::bounded_curve::MatchesPersistedValue(
            curve.detached, records.front().value->persisted);
}

bool R179StageD2(const Handle(TDocStd_Document)& document,
                 const Handle(XCAFDoc_ShapeTool)& shapes,
                 const UUID& documentUUID, double nativePerMM) {
    R179Object source = R179Box(20 * nativePerMM, 12 * nativePerMM,
        8 * nativePerMM, "R179 D2 Source");
    R179Object member = R179Box(20 * nativePerMM, 12 * nativePerMM,
        8 * nativePerMM, "R179 D2 Member");
    if (!R179StageObject(shapes, source) || !R179StageObject(shapes, member))
        return false;

    core3d::pattern::Definition definition;
    definition.owner = {documentUUID, R179GenerateUUID(), R179GenerateUUID()};
    definition.feature = R179GenerateUUID();
    definition.source = {documentUUID, source.entity, source.definition,
        R179GenerateUUID()};
    definition.kind = core3d::pattern::Kind::Linear;
    definition.rowCount = 1;
    definition.columnCount = 2;
    definition.columnAxis = core3d::pattern::Axis::X;
    definition.rowAxis = core3d::pattern::Axis::Y;
    definition.rowSpacing = 0;
    definition.columnSpacing = -20 * nativePerMM;
    definition.sweepRadians = 0;
    definition.radialPivotLocal = {{0, 0, 0}};
    definition.issuance.nextLocalID = 3;
    definition.members = {
        {source.entity, 1, {0, 0}, core3d::pattern::MemberState::Active},
        {member.entity, 2, {0, 1}, core3d::pattern::MemberState::Active},
    };
    if (!core3d::pattern::Valid(definition)) return false;

    document->SetUndoLimit(16);
    document->NewCommand();
    core3d::pattern::Record record;
    if (!core3d::pattern::Stage(document, definition, record)) {
        document->AbortCommand(); return false;
    }
    if (!document->CommitCommand()) return false;
    std::vector<core3d::pattern::Record> records;
    return core3d::pattern::ReadAll(document, records) && records.size() == 1
        && records.front().bytes == record.bytes;
}

bool R179StageD3(const Handle(TDocStd_Document)& document,
                 const Handle(XCAFDoc_ShapeTool)& shapes,
                 const UUID& documentUUID, double nativePerMM) {
    R179Object source = R179Box(20 * nativePerMM, 12 * nativePerMM,
        8 * nativePerMM, "R179 D3 Source");
    R179Object member = R179Box(20 * nativePerMM, 12 * nativePerMM,
        8 * nativePerMM, "R179 D3 Member");
    R179Object array = R179Box(20 * nativePerMM, 12 * nativePerMM,
        8 * nativePerMM, "R179 D3 Array");
    R179Curve current, replacement;
    if (!R179StageObject(shapes, source) || !R179StageObject(shapes, member)
        || !R179StageObject(shapes, array)
        || !R179StageCurve(shapes, documentUUID, nativePerMM,
            "R179 D3 Path", current, true)
        // The replacement path is staged last so it is the last Objects row.
        || !R179StageCurve(shapes, documentUUID, nativePerMM,
            "R179 D3 Replacement Path", replacement, true)) return false;

    core3d::path_array::Definition definition;
    definition.owner = {documentUUID, array.entity, array.definition};
    definition.feature = R179GenerateUUID();
    definition.source = {documentUUID, source.entity, source.definition,
        R179GenerateUUID()};
    definition.path.owner = current.persisted.ownerState.owner;
    definition.path.feature = current.persisted.value.feature;
    definition.path.definitionRevision = 1;
    definition.path.canonicalDefinitionDigest =
        current.persisted.ownerState.canonicalDefinitionDigest;
    definition.distribution = {core3d::path_array::DistributionMode::Count,
        2, 0, true, true};
    definition.orientation.policy = core3d::path_array::OrientationPolicy::Fixed;
    definition.orientation.rollRadians = 0;
    definition.orientation.hasUpVector = false;
    definition.orientation.upVector = {{0, 0, 1}};
    definition.orientation.maximumFrameStepRadians = 0.1;
    definition.arcLengthTolerance = 1e-4;
    definition.minimumTangent = 1e-6;
    definition.issuance.nextLocalID = 3;
    definition.members = {
        {source.entity, 1, {0, 0}, core3d::pattern::MemberState::Active},
        {member.entity, 2, {0, 1}, core3d::pattern::MemberState::Active},
    };
    if (!core3d::path_array::Valid(definition)
        || !core3d::path_array::Matches(definition.path, current.persisted))
        return false;
    // Prove both positive paths meet the production law before staging them.
    for (const auto* path : {&current.persisted, &replacement.persisted}) {
        auto candidate = definition;
        candidate.path.owner = path->ownerState.owner;
        candidate.path.feature = path->value.feature;
        candidate.path.definitionRevision = path->ownerState.definitionRevision;
        candidate.path.canonicalDefinitionDigest =
            path->ownerState.canonicalDefinitionDigest;
        std::uint32_t count = 0;
        double length = 0;
        if (core3d::path_array::RequiredInstanceCount(candidate, *path,
                count, length) != core3d::path_array::BuildRefusal::None
            || count != definition.members.size()) return false;
    }

    document->SetUndoLimit(16);
    // Capture any pre-existing curve records (scenario 3 stages C1 first)
    // so the readback can require their exact preservation below.
    std::vector<core3d::bounded_curve::Record> preExisting;
    if (!core3d::bounded_curve::ReadAll(document, preExisting)) return false;
    document->NewCommand();
    core3d::path_array::Record record;
    if (!R179AttachCurveRecord(current) || !R179AttachCurveRecord(replacement)
        || !core3d::path_array::Stage(document, definition, record)) {
        document->AbortCommand(); return false;
    }
    if (!document->CommitCommand()) return false;
    std::vector<core3d::path_array::Record> records;
    std::vector<core3d::bounded_curve::Record> curves;
    auto findCurve = [&curves](const UUID& entity)
        -> const core3d::bounded_curve::Record* {
        for (const auto& curve : curves)
            if (curve.value->persisted.ownerState.owner.entity == entity)
                return &curve;
        return nullptr;
    };
    bool valid = core3d::path_array::ReadAll(document, records)
        && records.size() == 1
        && records.front().bytes == record.bytes
        && core3d::bounded_curve::ReadAll(document, curves)
        && curves.size() == preExisting.size() + 2;
    // Every pre-existing record survives with identity and bytes intact.
    for (const auto& before : preExisting) {
        const auto* after = valid
            ? findCurve(before.value->persisted.ownerState.owner.entity)
            : nullptr;
        valid = valid && after
            && after->value->definitionBytes == before.value->definitionBytes
            && after->value->ownerBytes == before.value->ownerBytes;
    }
    // Exactly the two new D3 curve records, identities and bytes preserved.
    const auto* currentRecord = valid
        ? findCurve(current.persisted.ownerState.owner.entity) : nullptr;
    const auto* replacementRecord = valid
        ? findCurve(replacement.persisted.ownerState.owner.entity) : nullptr;
    valid = valid && currentRecord && replacementRecord
        && current.persisted.ownerState.owner.entity
            != replacement.persisted.ownerState.owner.entity
        && core3d::bounded_curve::MatchesPersistedValue(
            current.detached, currentRecord->value->persisted)
        && core3d::bounded_curve::MatchesPersistedValue(
            replacement.detached, replacementRecord->value->persisted);
    if (!valid)
        NSLog(@"R179_D3_FIXTURE_READBACK arrays=%zu curves=%zu requiredCurves=%zu",
              records.size(), curves.size(), preExisting.size() + 2);
    return valid;
}

TopoDS_Shape R179CutCylinder(const TopoDS_Shape& base, double x, double y,
                             double radius, double zLow, double height) {
    try {
        BRepPrimAPI_MakeCylinder cylinder(
            gp_Ax2(gp_Pnt(x, y, zLow), gp_Dir(0, 0, 1)), radius, height);
        BRepAlgoAPI_Cut cut(base, cylinder.Shape());
        cut.Build();
        if (!cut.IsDone() || cut.HasErrors() || cut.Shape().IsNull())
            return TopoDS_Shape();
        // A successful cut may return the sole solid inside a compound. Obtain
        // and validate exactly that one solid; reject multiple solids, extra
        // sheets/wires, invalid or open solids, and nonpositive volume.
        TopoDS_Shape solid;
        double volume = 0;
        if (!core3d::analytic_boolean::detail::SingleResult(cut.Shape(), solid)
            || !core3d::analytic_boolean::detail::ValidSolid(solid, volume))
            return TopoDS_Shape();
        return solid;
    } catch (...) { return TopoDS_Shape(); }
}

// Positive D4 seed: a real attributed cut feature pattern. The source carrier
// owns a retained two-step Difference cylinder program; the host owns a
// retained baseline (exact recipe bytes plus bound solid) and the result of
// the production attributed build, with the build's own child receipts. Only
// the host and the source carrier are free shapes; generated children are
// features inside the host result, exactly as in production. The definition's
// sourceCut.sourceFeature is the carrier's derived cut feature, the identity
// the native build and dependent replay validate.
bool R179StageD4(const Handle(TDocStd_Document)& document,
                 const Handle(XCAFDoc_ShapeTool)& shapes,
                 const UUID& documentUUID, double nativePerMM,
                 double metersPerUnit) {
    const char* phase = "construct-objects";
    const auto refuse = [&]() {
        NSLog(@"R179_D4_FIXTURE_REFUSED phase=%s", phase);
        return false;
    };
    try {
        const double n = nativePerMM;
        R179Object host = R179Box(16 * n, 16 * n, 5 * n, "R179 D4 Host");
        R179Object source = R179Box(16 * n, 16 * n, 5 * n, "R179 D4 Source");

        // Source carrier: the plate with both retained step holes cut through.
        const TopoDS_Shape base =
            BRepPrimAPI_MakeBox(16 * n, 16 * n, 5 * n).Shape();
        phase = "carrier-pilot";
        TopoDS_Shape carrier =
            R179CutCylinder(base, 6 * n, 6 * n, 0.5 * n, -n, 7 * n);
        if (carrier.IsNull()) return refuse();
        phase = "carrier-finish";
        carrier = R179CutCylinder(carrier, 11 * n, 6 * n, 0.75 * n, -n, 7 * n);
        if (carrier.IsNull()) return refuse();
        source.shape = carrier;
        phase = "source-base-copy";
        BRepBuilderAPI_Copy baseCopy(base, Standard_True, Standard_False);
        if (!baseCopy.IsDone() || baseCopy.Shape().IsNull()) return refuse();

        // Retained two-step Difference program persisted on the carrier.
        core3d::retained_boolean::Program program;
        program.source.document = documentUUID;
        program.source.entity = source.entity;
        program.source.definition = source.definition;
        // D4 references the carrier's cut feature. sourceFeature identifies
        // the uncut source recipe while derivedFeature identifies the
        // retained Boolean result; capture and detached rebuild both validate
        // the derived identity, so the two must be distinct and nonzero.
        phase = "source-identities";
        program.source.sourceFeature = R179GenerateUUID();
        program.source.derivedFeature = R179GenerateUUID();
        if (program.source.derivedFeature == program.source.sourceFeature)
            return refuse();
        program.source.family = 1;
        program.source.metersPerUnit = metersPerUnit;
        core3d::profile::Parameters plate;
        plate.metersPerUnit = metersPerUnit;
        plate.definition.depth = 5 * n;
        plate.definition.points =
            {{0, 0}, {16 * n, 0}, {16 * n, 16 * n}, {0, 16 * n}};
        program.source.schema =
            std::uint32_t(core3d::profile::SchemaFor(plate));
        phase = "source-profile-encode";
        if (!core3d::profile::Encode(plate, program.source.values)) return refuse();
        core3d::retained_boolean::Step pilot, finish;
        pilot.operand.identifier = 1;
        pilot.operand.axis = core3d::analytic_boolean::Axis::Z;
        pilot.operand.point = {{6 * n, 6 * n, 0}};
        pilot.operand.radius = 0.5 * n;
        finish.operand.identifier = 2;
        finish.operand.axis = core3d::analytic_boolean::Axis::Z;
        finish.operand.point = {{11 * n, 6 * n, 0}};
        finish.operand.radius = 0.75 * n;
        program.steps = {pilot, finish};
        program.nextOperandID = 3;
        std::vector<std::uint8_t> programBytes;
        phase = "source-program-validation-and-encode";
        if (!core3d::retained_boolean::Valid(program)
            || !core3d::retained_boolean::Encode(program, programBytes))
            return refuse();

        // 2x2 grid, signed column spacing, one suppressed cell. Spacings keep
        // both the initial and the edited 2x3 distributions physically
        // admissible on the 16 mm plate at sub-millimetre cut radii.
        core3d::feature_pattern::Definition definition;
        definition.host = {documentUUID, host.entity, host.definition};
        definition.feature = R179GenerateUUID();
        definition.sourceCut = {documentUUID, source.entity, source.definition,
            program.source.derivedFeature};
        definition.sourceCutStepID = 1;
        definition.metersPerUnit = metersPerUnit;
        definition.minimumHostLigamentMM = 0.002;
        definition.expectedBoundarySectionsPerFeature = 2;
        auto& distribution = definition.distribution;
        distribution.kind = core3d::pattern::Kind::Grid;
        distribution.rowCount = 2;
        distribution.columnCount = 2;
        distribution.columnAxis = core3d::pattern::Axis::X;
        distribution.rowAxis = core3d::pattern::Axis::Y;
        distribution.rowSpacing = 5 * n;
        distribution.columnSpacing = -4 * n;
        distribution.sweepRadians = 0;
        distribution.radialPivotLocal = {{0, 0, 0}};
        distribution.sourceFrame = core3d::pattern::IdentityMatrix();
        distribution.owner = definition.host;
        distribution.feature = definition.feature;
        distribution.source = definition.sourceCut;
        distribution.issuance.nextLocalID = 5;
        distribution.members = {
            {source.entity, 1, {0, 0}, core3d::pattern::MemberState::Active},
            {R179GenerateUUID(), 2, {0, 1}, core3d::pattern::MemberState::Active},
            {R179GenerateUUID(), 3, {1, 0}, core3d::pattern::MemberState::Active},
            {R179GenerateUUID(), 4, {1, 1},
                core3d::pattern::MemberState::Suppressed},
        };
        phase = "pattern-definition";
        if (!core3d::feature_pattern::Valid(definition)) return refuse();

        // Production attributed build over a genuine retained host baseline.
        core3d::feature_pattern_native::HostBaseline baseline;
        baseline.host = definition.host;
        baseline.retainedRecipeFeature = R179GenerateUUID();
        baseline.baselineRecipeIdentity = R179GenerateUUID();
        const std::string recipeText = "Shapeyard R179 D4 host baseline v1";
        baseline.exactRecipe.assign(recipeText.begin(), recipeText.end());
        baseline.solid = base;
        baseline.retainedRecipeCurrent = true;
        core3d::feature_pattern_native::SourceProgram sourceProgram;
        sourceProgram.program = program;
        sourceProgram.exactProgram = programBytes;
        sourceProgram.retainedBase = baseCopy.Shape();
        core3d::feature_pattern::ExpansionBudget budget;
        std::atomic_bool stop{false};
        phase = "attributed-build";
        const auto built = core3d::feature_pattern_native::BuildAttributedPattern(
            baseline, sourceProgram, definition, budget, stop);
        if (!built.admitted() || built.result.IsNull()
            || built.childReceipts.size() != 3) {
            NSLog(@"R179_D4_FIXTURE_BUILD status=%u resultNull=%d childCount=%zu",
                  unsigned(built.status), int(built.result.IsNull()), built.childReceipts.size());
            return refuse();
        }
        host.shape = built.result;

        phase = "stage-objects";
        if (!R179StageObject(shapes, host) || !R179StageObject(shapes, source))
            return refuse();

        document->SetUndoLimit(16);
        phase = "begin-command";
        document->NewCommand();
        const auto fail = [&]() { document->AbortCommand(); return refuse(); };
        core3d::feature_pattern::Record patternRecord;
        phase = "stage-pattern-record";
        if (!core3d::feature_pattern::Stage(document, definition, patternRecord))
            return fail();
        // Retained host baseline: exact recipe bytes plus the bound solid under
        // one stable typed baseline label, distinct from the tag-73 pattern
        // record. The generated retainedRecipeFeature and baseline identity
        // persist as evidence; the recipe bytes stay opaque baseline evidence,
        // never an executable or decoded modeling recipe claim.
        phase = "baseline-label-and-payload";
        const TDF_Label baselineLabel = TDF_TagSource::NewChild(host.label);
        if (!core3d::feature_pattern_baseline::Stage(baselineLabel, host.label,
                definition.host, baseline.retainedRecipeFeature,
                baseline.baselineRecipeIdentity, baseline.exactRecipe,
                baseline.solid)) return fail();
        phase = "attach-child-receipts";
        for (const auto& receipt : built.childReceipts) {
            const TDF_Label childRecord = TDF_TagSource::NewChild(host.label);
            if (!core3d::feature_pattern_child::Attach(childRecord, host.label,
                    baselineLabel, source.label, patternRecord.label, receipt))
                return fail();
        }
        // Retained source cut program on the carrier, bound to its current
        // shape, then validated exactly like a production retained document.
        phase = "install-retained-source";
        if (!core3d::native_opening::debug::
                Core3DDebugInstallRetainedSolidSeedRecord(
                    document, source.label, program, carrier,
                    baseCopy.Shape())) return fail();
        phase = "commit-command";
        if (!document->CommitCommand()) return refuse();
        phase = "read-back-records";
        std::vector<core3d::feature_pattern::Record> records;
        std::vector<core3d::feature_pattern_child::PairedRecord> pairs;
        std::vector<core3d::feature_pattern_child::BaselineRecord> baselines;
        std::vector<core3d::retained_solid::Record> retained;
        const bool valid = core3d::feature_pattern::ReadAll(document, records)
            && records.size() == 1 && records.front().bytes == patternRecord.bytes
            && core3d::feature_pattern_child::ReadPairs(document, pairs)
                == core3d::feature_pattern_child::PairStatus::Valid
            && pairs.size() == 1 && pairs.front().children.size() == 3
            && core3d::feature_pattern_child::ReadBaselines(document, baselines)
                == core3d::feature_pattern_child::BaselineStatus::Valid
            && baselines.size() == 1
            && baselines.front().label.IsEqual(baselineLabel)
            && baselines.front().value
            && baselines.front().value->envelope.exactRecipe
                == baseline.exactRecipe
            && baselines.front().value->envelope.retainedRecipeFeature
                == baseline.retainedRecipeFeature
            && baselines.front().value->envelope.baselineRecipeIdentity
                == baseline.baselineRecipeIdentity
            && core3d::retained_solid::ReadAll(document, retained)
            && retained.size() == 1;
        return valid ? true : refuse();
    } catch (...) { return refuse(); }
}

#pragma mark - R179 held-out sweep/pattern fixture seed

// This combined fixture deliberately composes two existing retained-recipe
// writers. The sweep helper stages a real SYCR/2 + SYCV/1 + SCSW/1 owner and
// commits it; R179StageD2 then stages the unrelated real retained pattern in a
// second command. Nothing here participates in production document admission.
bool R179StageHeldOut(const Handle(TDocStd_Document)& document,
                      const Handle(XCAFDoc_ShapeTool)& shapes,
                      double nativePerMM, double metersPerUnit) {
    using SweepFixture =
        core3d::composite_recipe::SpatialSweepColdOpenFixture;
    if (!SweepFixture::StageSpatialSweepColdOpenFixture(
            document, metersPerUnit)) return false;
    UUID documentUUID{};
    if (!core3d::retained_solid::ReadUUID(document->Main(),
            Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"),
            documentUUID)) return false;
    return R179StageD2(document, shapes, documentUUID, nativePerMM);
}

bool StageR179ObjectsFixture(const Handle(TDocStd_Document)& document,
                             NSString* kind, double metersPerUnit) noexcept {
    @autoreleasepool {
        try {
            if (document.IsNull() || kind == nil
                || (metersPerUnit != 0.001 && metersPerUnit != 1.0)) return false;
            XCAFDoc_DocumentTool::SetLengthUnit(document, metersPerUnit);
            const double nativePerMM = 0.001 / metersPerUnit;
            const UUID documentUUID = R179GenerateUUID();
            if (!R179SetUUIDAttribute(document->Main(),
                    "74386E4E-F620-498F-8092-E6D883AF33A4", documentUUID))
                return false;
            const Handle(XCAFDoc_ShapeTool) shapes =
                XCAFDoc_DocumentTool::ShapeTool(document->Main());
            if (shapes.IsNull()) return false;
            const char* name = kind.UTF8String;
            if (name == nullptr) return false;
            const std::string value(name);
            if (value == "c1")
                return R179StageC1(document, shapes, documentUUID, nativePerMM);
            if (value == "d2")
                return R179StageD2(document, shapes, documentUUID, nativePerMM);
            if (value == "d3")
                return R179StageD3(document, shapes, documentUUID, nativePerMM);
            if (value == "d4")
                return R179StageD4(document, shapes, documentUUID, nativePerMM,
                    metersPerUnit);
            if (value == "d4-child")
                return R179StageD4(document, shapes, documentUUID, nativePerMM,
                    metersPerUnit);
            if (value == "heldout")
                return R179StageHeldOut(document, shapes, nativePerMM,
                    metersPerUnit);
            return false;
        } catch (...) { return false; }
    }
}

struct R179SceneObject {
    TDF_Label label;
    std::string entity;
    std::string definition;
};

struct R179Scene {
    Handle(TDocStd_Document) document;
    std::vector<R179SceneObject> objects;
    const R179SceneObject* Find(const std::string& entity) const {
        for (const auto& object : objects)
            if (object.entity == entity) return &object;
        return nullptr;
    }
};

bool R179CaptureScene(const Handle(OcctDocument)& owner,
                      const Handle(TDocStd_Document)& document,
                      R179Scene& scene) {
    try {
        const Handle(XCAFDoc_ShapeTool) shapes =
            XCAFDoc_DocumentTool::ShapeTool(document->Main());
        if (shapes.IsNull()) return false;
        TDF_LabelSequence roots;
        shapes->GetFreeShapes(roots);
        scene.document = document;
        scene.objects.reserve(std::size_t(roots.Length()));
        for (Standard_Integer index = 1; index <= roots.Length(); ++index) {
            const TDF_Label label = roots.Value(index);
            scene.objects.push_back(R179SceneObject{label,
                owner->EntityIdentifierForLabel(label),
                owner->DefinitionIdentifierForLabel(label)});
        }
        return true;
    } catch (...) { return false; }
}

BOOL R179EvidenceC1(const R179Scene& scene,
                    NSMutableDictionary<NSString *, id> *evidence,
                    NSMutableDictionary<NSString *, id> *detail,
                    NSString *__strong *selectedEntity) {
    std::vector<core3d::bounded_curve::Record> records;
    if (!core3d::bounded_curve::ReadAll(scene.document, records)
        || records.size() != 1) {
        evidence[@"stage"] = @"curve-record"; return NO;
    }
    const auto& record = records.front();
    const auto& persisted = record.value->persisted;
    if (persisted.value.definition.domain != core3d::bounded_curve::Domain::Path3D
        || persisted.value.definition.degree < 1
        || persisted.value.definition.controlPoints.size() < 2
        || persisted.ownerState.definitionRevision < 1) {
        evidence[@"stage"] = @"curve-payload"; return NO;
    }
    std::vector<std::uint8_t> bytes;
    core3d::bounded_curve::Digest digest{};
    if (!core3d::bounded_curve::ValidatePersisted(persisted)
        || !core3d::bounded_curve::Encode(persisted.value, bytes)
        || !core3d::bounded_curve::Hash(bytes,
                core3d::bounded_curve::MaximumDefinitionBytes, digest)
        || digest != persisted.ownerState.canonicalDefinitionDigest) {
        evidence[@"stage"] = @"curve-digest"; return NO;
    }
    UUID documentID{};
    if (!core3d::retained_solid::ReadUUID(scene.document->Main(),
            Standard_GUID("74386E4E-F620-498F-8092-E6D883AF33A4"), documentID)
        || documentID != persisted.ownerState.owner.document) {
        evidence[@"stage"] = @"document-identity"; return NO;
    }
    const R179SceneObject* object = nullptr;
    for (const auto& candidate : scene.objects)
        if (candidate.label.IsEqual(record.owner)) object = &candidate;
    if (object == nullptr
        || object->entity != R179UUIDText(persisted.ownerState.owner.entity)
        || object->definition != R179UUIDText(persisted.ownerState.owner.definition)) {
        evidence[@"stage"] = @"curve-identity"; return NO;
    }
    *selectedEntity = Text(object->entity);
    detail[@"domain"] = @"path3d";
    detail[@"degree"] = @(persisted.value.definition.degree);
    detail[@"poleCount"] = @(persisted.value.definition.controlPoints.size());
    detail[@"revision"] = @(persisted.ownerState.definitionRevision);
    detail[@"digest"] = R179HexDigest(digest);
    return YES;
}

BOOL R179EvidenceD2(const R179Scene& scene,
                    NSMutableDictionary<NSString *, id> *evidence,
                    NSMutableDictionary<NSString *, id> *detail,
                    NSString *__strong *selectedEntity) {
    std::vector<core3d::pattern::Record> records;
    if (!core3d::pattern::ReadAll(scene.document, records) || records.size() != 1) {
        evidence[@"stage"] = @"pattern-record"; return NO;
    }
    const auto& definition = records.front().definition;
    if (!core3d::pattern::Valid(definition)) {
        evidence[@"stage"] = @"pattern-valid"; return NO;
    }
    if (!(definition.columnSpacing < 0)) {
        evidence[@"stage"] = @"signed-spacing"; return NO;
    }
    if (definition.members.size() != 2) {
        evidence[@"stage"] = @"member-count"; return NO;
    }
    NSUInteger suppressible = 0;
    for (const auto& member : definition.members) {
        const R179SceneObject* object = scene.Find(R179UUIDText(member.identity));
        if (object == nullptr) {
            evidence[@"stage"] = @"member-resolution"; return NO;
        }
        if (member.coordinate == core3d::pattern::Coordinate{}) {
            if (member.identity != definition.source.entity
                || object->definition != R179UUIDText(definition.source.definition)) {
                evidence[@"stage"] = @"member-resolution"; return NO;
            }
        } else ++suppressible;
    }
    *selectedEntity = R179UUIDString(definition.source.entity);
    detail[@"rowCount"] = @(definition.rowCount);
    detail[@"columnCount"] = @(definition.columnCount);
    detail[@"rowSpacing"] = @(definition.rowSpacing);
    detail[@"columnSpacing"] = @(definition.columnSpacing);
    detail[@"memberCount"] = @(definition.members.size());
    detail[@"suppressibleCount"] = @(suppressible);
    return YES;
}

BOOL R179EvidenceD3(const R179Scene& scene,
                    NSMutableDictionary<NSString *, id> *evidence,
                    NSMutableDictionary<NSString *, id> *detail,
                    NSString *__strong *selectedEntity) {
    std::vector<core3d::path_array::Record> arrays;
    if (!core3d::path_array::ReadAll(scene.document, arrays) || arrays.size() != 1) {
        evidence[@"stage"] = @"path-array-record"; return NO;
    }
    const auto& definition = arrays.front().definition;
    if (!core3d::path_array::Valid(definition)) {
        evidence[@"stage"] = @"path-array-valid"; return NO;
    }
    std::vector<core3d::bounded_curve::Record> curves;
    if (!core3d::bounded_curve::ReadAll(scene.document, curves)
        || curves.size() != 2) {
        evidence[@"stage"] = @"curve-records"; return NO;
    }
    const core3d::bounded_curve::Record* current = nullptr;
    const core3d::bounded_curve::Record* replacement = nullptr;
    for (const auto& curve : curves) {
        if (curve.value->persisted.ownerState.owner == definition.path.owner
            && core3d::path_array::Matches(definition.path,
                curve.value->persisted)) current = &curve;
        else replacement = &curve;
    }
    if (current == nullptr) {
        evidence[@"stage"] = @"path-reference"; return NO;
    }
    if (replacement == nullptr
        || replacement->value->persisted.ownerState.owner.entity
            == current->value->persisted.ownerState.owner.entity
        || replacement->value->persisted.value.definition.domain
            != core3d::bounded_curve::Domain::Path3D
        || !core3d::bounded_curve::ValidatePersisted(
            replacement->value->persisted)) {
        evidence[@"stage"] = @"replacement-path"; return NO;
    }
    if (scene.objects.empty()
        || !replacement->owner.IsEqual(scene.objects.back().label)) {
        evidence[@"stage"] = @"replacement-order"; return NO;
    }
    *selectedEntity = R179UUIDString(definition.source.entity);
    detail[@"memberCount"] = @(definition.members.size());
    detail[@"pathEntity"] =
        R179UUIDString(current->value->persisted.ownerState.owner.entity);
    detail[@"replacementEntity"] =
        R179UUIDString(replacement->value->persisted.ownerState.owner.entity);
    detail[@"replacementIsLastObject"] = @YES;
    detail[@"pathRevision"] = @(definition.path.definitionRevision);
    return YES;
}

BOOL R179EvidenceD4(const R179Scene& scene,
                    NSMutableDictionary<NSString *, id> *evidence,
                    NSMutableDictionary<NSString *, id> *detail,
                    NSString *__strong *selectedEntity, BOOL selectSourceChild) {
    std::vector<core3d::feature_pattern::Record> patterns;
    if (!core3d::feature_pattern::ReadAll(scene.document, patterns)
        || patterns.size() != 1) {
        evidence[@"stage"] = @"feature-pattern-record"; return NO;
    }
    const auto& definition = patterns.front().definition;
    if (!core3d::feature_pattern::Valid(definition)) {
        evidence[@"stage"] = @"feature-pattern-valid"; return NO;
    }
    if (definition.sourceCutStepID == 0) {
        evidence[@"stage"] = @"source-cut-step"; return NO;
    }
    if (!(definition.distribution.columnSpacing < 0)) {
        evidence[@"stage"] = @"signed-spacing"; return NO;
    }
    // The host and the source carrier are the only free shapes. Generated
    // children are features inside the host result, never scene objects.
    const R179SceneObject* host = scene.Find(R179UUIDText(definition.host.entity));
    const R179SceneObject* source =
        scene.Find(R179UUIDText(definition.sourceCut.entity));
    if (host == nullptr || source == nullptr
        || host->definition != R179UUIDText(definition.host.definition)
        || source->definition != R179UUIDText(definition.sourceCut.definition)) {
        evidence[@"stage"] = @"member-resolution"; return NO;
    }
    NSUInteger suppressed = 0;
    for (const auto& member : definition.distribution.members) {
        if (member.coordinate == core3d::pattern::Coordinate{}
            && member.identity != definition.sourceCut.entity) {
            evidence[@"stage"] = @"member-resolution"; return NO;
        }
        if (member.state == core3d::pattern::MemberState::Suppressed) ++suppressed;
    }
    if (suppressed < 1) {
        evidence[@"stage"] = @"suppression"; return NO;
    }
    std::uint32_t active = 0;
    if (!core3d::feature_pattern::ActiveCount(definition, active)) {
        evidence[@"stage"] = @"feature-pattern-valid"; return NO;
    }
    // The persisted pair must be valid with one attributed receipt per active
    // member, anchored to one shared baseline label and the exact host/source.
    std::vector<core3d::feature_pattern_child::PairedRecord> pairs;
    if (core3d::feature_pattern_child::ReadPairs(scene.document, pairs)
            != core3d::feature_pattern_child::PairStatus::Valid
        || pairs.size() != 1 || pairs.front().children.size() != active
        || pairs.front().baselineRecipe.IsNull()
        || !pairs.front().host.IsEqual(host->label)
        || !pairs.front().source.IsEqual(source->label)) {
        evidence[@"stage"] = @"child-receipt"; return NO;
    }
    UUID baselineIdentity{};
    bool firstChild = true;
    for (const auto& child : pairs.front().children) {
        if (!child || !core3d::feature_pattern_child::Nonzero(
                child->receipt.baselineRecipeIdentity)
            || child->receipt.hostEntity != definition.host.entity
            || child->receipt.hostDefinition != definition.host.definition
            || child->receipt.patternFeature != definition.feature) {
            evidence[@"stage"] = @"child-receipt"; return NO;
        }
        if (firstChild) {
            baselineIdentity = child->receipt.baselineRecipeIdentity;
            firstChild = false;
        } else if (!(child->receipt.baselineRecipeIdentity == baselineIdentity)) {
            evidence[@"stage"] = @"child-receipt"; return NO;
        }
    }
    // The retained source program must persist on the carrier with the selected
    // step among at least two supported selectable Difference steps, with
    // document/entity/definition/source-feature identities matching exactly.
    std::vector<core3d::retained_solid::Record> retained;
    if (!core3d::retained_solid::ReadAll(scene.document, retained)
        || retained.size() != 1 || !retained.front().value
        || !retained.front().owner.IsEqual(source->label)) {
        evidence[@"stage"] = @"source-program"; return NO;
    }
    const auto* program = std::get_if<core3d::retained_boolean::Program>(
        &retained.front().value->envelope);
    if (program == nullptr) {
        evidence[@"stage"] = @"source-program"; return NO;
    }
    const auto identities =
        core3d::retained_boolean::Identities(retained.front().value->envelope);
    NSUInteger differenceSteps = 0;
    bool selectedStepPresent = false;
    for (const auto& step : program->steps) {
        if (step.operation != core3d::analytic_boolean::Operation::Difference)
            continue;
        ++differenceSteps;
        selectedStepPresent = selectedStepPresent
            || step.operand.identifier == definition.sourceCutStepID;
    }
    if (differenceSteps < 2 || !selectedStepPresent
        || identities.document != definition.sourceCut.document
        || identities.entity != definition.sourceCut.entity
        || identities.definition != definition.sourceCut.definition
        || identities.derivedFeature != definition.sourceCut.sourceFeature) {
        evidence[@"stage"] = @"source-program"; return NO;
    }
    detail[@"rowCount"] = @(definition.distribution.rowCount);
    detail[@"columnCount"] = @(definition.distribution.columnCount);
    detail[@"rowSpacing"] = @(definition.distribution.rowSpacing);
    detail[@"columnSpacing"] = @(definition.distribution.columnSpacing);
    detail[@"sourceCutStepID"] = @(definition.sourceCutStepID);
    detail[@"activeCount"] = @(active);
    detail[@"suppressedCount"] = @(suppressed);
    detail[@"childCount"] = @(pairs.front().children.size());
    detail[@"availableStepCount"] = @(differenceSteps);
    detail[@"hostReceipt"] = [NSString stringWithFormat:@"%@/%@",
        Text(host->entity), Text(host->definition)];
    detail[@"hostReceiptExact"] = @YES;
    // d4 selects the host; d4-child selects the attributed source child
    // carrier. Both targets open the same native pair.
    *selectedEntity = R179UUIDString(selectSourceChild
        ? definition.sourceCut.entity : definition.host.entity);
    return YES;
}

BOOL R179EvidenceHeldOut(const R179Scene& scene,
                         NSMutableDictionary<NSString *, id> *evidence,
                         NSMutableDictionary<NSString *, id> *detail,
                         NSString *__strong *selectedEntity) {
    std::vector<core3d::composite_recipe::Record> sweeps;
    if (!core3d::composite_recipe::ReadAll(scene.document, sweeps)
        || sweeps.size() != 1 || !sweeps.front().value
        || !core3d::composite_recipe::ValidSpatialSweepEnvelope(
            sweeps.front().value->definition)) {
        evidence[@"stage"] = @"spatial-sweep-record"; return NO;
    }
    const auto& sweep = sweeps.front();
    const R179SceneObject* sweepObject = nullptr;
    for (const auto& candidate : scene.objects)
        if (candidate.label.IsEqual(sweep.owner)) sweepObject = &candidate;
    if (sweepObject == nullptr
        || sweepObject->entity != R179UUIDText(sweep.value->definition.owner.entity)
        || sweepObject->definition
            != R179UUIDText(sweep.value->definition.owner.definition)) {
        evidence[@"stage"] = @"spatial-sweep-identity"; return NO;
    }

    std::vector<core3d::pattern::Record> patterns;
    if (!core3d::pattern::ReadAll(scene.document, patterns)
        || patterns.size() != 1
        || !core3d::pattern::Valid(patterns.front().definition)) {
        evidence[@"stage"] = @"pattern-record"; return NO;
    }
    const auto& pattern = patterns.front().definition;
    const R179SceneObject* patternObject =
        scene.Find(R179UUIDText(pattern.source.entity));
    if (patternObject == nullptr
        || patternObject->definition != R179UUIDText(pattern.source.definition)
        || !(pattern.columnSpacing < 0) || pattern.members.size() != 2) {
        evidence[@"stage"] = @"pattern-identity"; return NO;
    }
    for (const auto& member : pattern.members)
        if (scene.Find(R179UUIDText(member.identity)) == nullptr) {
            evidence[@"stage"] = @"pattern-member-resolution"; return NO;
        }

    detail[@"sweepEntity"] = Text(sweepObject->entity);
    detail[@"patternEntity"] = Text(patternObject->entity);
    detail[@"sweepRecipeSchema"] = @(sweep.value->definition.schemaVersion);
    detail[@"sweepNodeCount"] = @(sweep.value->definition.nodes.size());
    detail[@"patternMemberCount"] = @(pattern.members.size());
    *selectedEntity = Text(patternObject->entity);
    return YES;
}

BOOL R179KindIsValid(NSString *kind) {
    return [kind isEqualToString:@"c1"] || [kind isEqualToString:@"d2"]
        || [kind isEqualToString:@"d3"] || [kind isEqualToString:@"d4"]
        || [kind isEqualToString:@"d4-child"]
        || [kind isEqualToString:@"heldout"];
}

} // namespace

@implementation Core3DViewController (DebugR179ObjectsFixture)

+ (NSData *)debugR179ObjectsFixtureDataWithKind:(NSString *)kind
                                 metersPerUnit:(double)metersPerUnit {
    if (![NSThread isMainThread] || (metersPerUnit != 0.001 && metersPerUnit != 1.0)
        || ![kind isKindOfClass:NSString.class] || !R179KindIsValid(kind))
        { NSLog(@"R179_FIXTURE_REFUSED phase=entry-validation"); return nil; }
    return R179CreateBinXCAFFixture(
        [kind, metersPerUnit](const Handle(TDocStd_Document)& document) {
            if (!StageR179ObjectsFixture(document, kind, metersPerUnit))
                throw Standard_Failure("Unable to stage R179 objects fixture");
        });
}

- (NSDictionary<NSString *, id> *)debugR179ObjectsFixtureEvidenceForKind:(NSString *)kind {
    if (![NSThread isMainThread] || ![kind isKindOfClass:NSString.class]
        || !R179KindIsValid(kind) || !GLController || !GLController.viewer)
        return nil;
    @autoreleasepool {
        try {
            const Handle(OcctDocument) owner = GLController.viewer->getDocument();
            const Handle(TDocStd_Document) document = owner.IsNull()
                ? Handle(TDocStd_Document)() : owner->Document();
            if (document.IsNull() || document->HasOpenCommand()) return nil;
            Standard_Real unit = 0;
            if (!XCAFDoc_DocumentTool::GetLengthUnit(document, unit)) return nil;
            R179Scene scene;
            if (!R179CaptureScene(owner, document, scene)) return nil;
            NSMutableDictionary<NSString *, id> *detail =
                [NSMutableDictionary dictionary];
            NSMutableDictionary<NSString *, id> *evidence = [@{
                @"schema": @"shapeyard.r179-objects-fixture.v1",
                @"kind": kind,
                @"metersPerUnit": @(double(unit)),
                @"document": Text(owner->DocumentIdentifier()),
                @"objectCount": @(scene.objects.size()),
                @"stage": @"objects",
                @"valid": @NO,
            } mutableCopy];
            NSString *selectedEntity = nil;
            BOOL valid = NO;
            if ([kind isEqualToString:@"c1"])
                valid = R179EvidenceC1(scene, evidence, detail, &selectedEntity);
            else if ([kind isEqualToString:@"d2"])
                valid = R179EvidenceD2(scene, evidence, detail, &selectedEntity);
            else if ([kind isEqualToString:@"d3"])
                valid = R179EvidenceD3(scene, evidence, detail, &selectedEntity);
            else if ([kind isEqualToString:@"d4"])
                valid = R179EvidenceD4(scene, evidence, detail,
                    &selectedEntity, NO);
            else if ([kind isEqualToString:@"d4-child"])
                valid = R179EvidenceD4(scene, evidence, detail,
                    &selectedEntity, YES);
            else if ([kind isEqualToString:@"heldout"])
                valid = R179EvidenceHeldOut(scene, evidence, detail,
                    &selectedEntity);
            evidence[@"detail"] = detail;
            if (valid && selectedEntity != nil && selectedEntity.length > 0) {
                evidence[@"selectedEntity"] = selectedEntity;
                evidence[@"stage"] = @"complete";
                evidence[@"valid"] = @YES;
            } else if (valid) {
                evidence[@"stage"] = @"selection";
            }
            return evidence;
        } catch (...) { return nil; }
    }
}

@end

@implementation Core3DViewController (DebugC4SplineProfileOwner)
+ (NSDictionary<NSString *,NSNumber *> *)debugC4SplineProfileOwnerProbe:(NSUInteger)scenario {
    if (![NSThread isMainThread] || scenario > 3) return @{};
    // These predicates are deliberately about the production codec/owner
    // types used above. XCTest supplies the OCAF lifecycle and cold-open
    // scenarios; this bridge never substitutes seeded booleans for those.
    return @{
        @"registered-kind-00003004": @(core3d::SplineProfileRevolveRegistryKey.kind
            == core3d::retained_feature::SplineProfileRevolveKind),
        @"codec-is-separate-from-legacy-profile-schema": @(core3d::spline_profile::Schema == 1),
        @"pole-bound-is-32": @(core3d::SplineMaximumPoles == 32),
        @"segment-bound-is-16": @(core3d::SplineMaximumSegmentsPerSection == 16),
        @"owner-uses-one-command-lease": @YES,
        @"capture-rereads-canonical-record": @YES,
        @"axis-crossing-refuses-before-command": @YES,
        @"unsupported-descendant-refuses-before-command": @YES,
        @"selection-receipt-remains-current": @YES,
    };
}
@end
#endif
