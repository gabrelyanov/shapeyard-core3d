#pragma once

// Header-only, authority-free conversion between persisted native source
// values and immutable public DTOs. No document, viewer, shape or history
// object crosses this boundary.
#import <Core3D/Core3DSharedModelingValues.h>

#include "../OCCTKit/ProfilePersistence.hxx"

namespace core3d::source_value_adapter {

inline Core3DConstructionFrame *Frame(const profile::ConstructionFrame& value) {
    return [[Core3DConstructionFrame alloc]
        initWithTranslationX:value.values[0]
        translationY:value.values[1]
        translationZ:value.values[2]
        quaternionX:value.values[3]
        quaternionY:value.values[4]
        quaternionZ:value.values[5]
        quaternionW:value.values[6]
        signedUniformScale:value.values[7]];
}

inline Core3DConstructionFrame *Frame(
    const std::optional<profile::ConstructionFrame>& value) {
    return value ? Frame(*value) : nil;
}

inline bool Frame(Core3DConstructionFrame *value,
    profile::ConstructionFrame& result) noexcept {
    result = {};
    if (![value isMemberOfClass:Core3DConstructionFrame.class]) return false;
    result.values = {value.translationX,value.translationY,value.translationZ,
        value.quaternionX,value.quaternionY,value.quaternionZ,value.quaternionW,
        value.signedUniformScale};
    return result.IsValid();
}

inline NSArray<NSNumber *> *FrameValues(
    const std::optional<profile::ConstructionFrame>& value) {
    if (!value) return @[];
    NSMutableArray<NSNumber *> *result = [NSMutableArray arrayWithCapacity:8];
    for (double scalar : value->values) [result addObject:@(scalar)];
    return [result copy];
}

inline NSArray<Core3DProfileShellStep *> *Shells(
    const std::vector<profile::ShellStep>& values) {
    NSMutableArray<Core3DProfileShellStep *> *result =
        [NSMutableArray arrayWithCapacity:values.size()];
    for (const auto& value : values) {
        NSMutableArray<Core3DProfileShellOpening *> *openings =
            [NSMutableArray arrayWithCapacity:value.openings.size()];
        for (int key : value.openings) {
            auto opening = [[Core3DProfileShellOpening alloc]
                initWithAxis:Core3DProfileShellAxis(key / 2)
                side:Core3DProfileShellSide(key % 2)];
            if (!opening) return nil;
            [openings addObject:opening];
        }
        auto frame = Frame(value.frame);
        auto step = frame ? [[Core3DProfileShellStep alloc]
            initWithThickness:value.thickness
            metersPerLocalUnit:value.metersPerLocalUnit
            constructionFrame:frame openings:openings] : nil;
        if (!step) return nil;
        [result addObject:step];
    }
    return [result copy];
}

inline bool Shells(NSArray<Core3DProfileShellStep *> *values,
    std::vector<profile::ShellStep>& result) noexcept {
    result.clear();
    if (![values isKindOfClass:NSArray.class]
        || values.count > profile::MaximumShellSteps) return false;
    try {
        result.reserve(values.count);
        for (id object in values) {
            if (![object isMemberOfClass:Core3DProfileShellStep.class]) return false;
            Core3DProfileShellStep *value = object;
            profile::ShellStep native;
            native.thickness = value.thickness;
            native.metersPerLocalUnit = value.metersPerLocalUnit;
            if (!Frame(value.constructionFrame,native.frame)) return false;
            for (id openingObject in value.openings) {
                if (![openingObject isMemberOfClass:Core3DProfileShellOpening.class]) return false;
                Core3DProfileShellOpening *opening = openingObject;
                native.openings.push_back(int(opening.axis) * 2 + int(opening.side));
            }
            if (!native.IsValid()) return false;
            result.push_back(std::move(native));
        }
        return true;
    } catch (...) {
        result.clear();
        return false;
    }
}

} // namespace core3d::source_value_adapter
