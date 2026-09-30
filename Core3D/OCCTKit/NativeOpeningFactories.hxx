#pragma once

#import "../UI/Core3DModelingTypes.h"

#include "NativeOpeningContext.hxx"
#include "OcctDocument.h"

#include <Standard_Handle.hxx>

#include <memory>
#include <string>

namespace core3d::native_opening {

//! The controller obtains Context from Core3DViewer and passes it through
//! unchanged. These factories never reconstruct authority from DTO fields.
Core3DBoundedCurveEditingOpening* _Nullable OpenBoundedCurve(
    const Handle(OcctDocument)&, const std::string& selectedEntity,
    const std::shared_ptr<Context>&) noexcept;
//! Ordinary C1 authoring. No selection is required; the candidate carries
//! explicit construction values only and every durable identity is native.
Core3DBoundedCurveCreationOpening* _Nullable OpenBoundedCurveCreation(
    const Handle(OcctDocument)&, const std::shared_ptr<Context>&) noexcept;
Core3DPatternEditingOpening* _Nullable OpenPattern(
    const Handle(OcctDocument)&, const std::string& selectedEntity,
    const std::shared_ptr<Context>&) noexcept;
Core3DPathArrayEditingOpening* _Nullable OpenPathArray(
    const Handle(OcctDocument)&, const std::string& selectedEntity,
    const std::shared_ptr<Context>&) noexcept;
Core3DFeaturePatternEditingOpening* _Nullable OpenFeaturePattern(
    const Handle(OcctDocument)&, const std::string& selectedEntity,
    const std::shared_ptr<Context>&) noexcept;
Core3DGeneralLoftEditingOpening* _Nullable OpenGeneralLoft(
    const Handle(OcctDocument)&, const std::string& selectedEntity,
    const std::shared_ptr<Context>&) noexcept;
Core3DGeneralLoftEditingOpening* _Nullable BeginGeneralLoft(
    const Handle(OcctDocument)&, NSArray<NSDictionary<NSString *, id> *> *stations,
    NSArray<NSNumber *> *orderAxis, NSString *requestedName,
    const std::shared_ptr<Context>&) noexcept;

} // namespace core3d::native_opening
