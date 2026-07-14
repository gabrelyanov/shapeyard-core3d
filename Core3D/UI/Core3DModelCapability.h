//
//  Core3DModelCapability.h
//  Core3D
//
//  Stable model-representation feature policy shared by public Core3D APIs.
//

#import <Foundation/Foundation.h>

//! Stable feature policy for a model representation. These bit values are
//! public API and must never be renumbered or reused.
typedef NS_OPTIONS(NSUInteger, Core3DModelCapability) {
    Core3DModelCapabilityNone = 0,
    Core3DModelCapabilityObjectSelection = 1UL << 0,
    Core3DModelCapabilitySubshapeSelection = 1UL << 1,
    Core3DModelCapabilityTranslate = 1UL << 2,
    Core3DModelCapabilityRotate = 1UL << 3,
    Core3DModelCapabilityUniformScale = 1UL << 4,
    Core3DModelCapabilityNonuniformScale = 1UL << 5,
    Core3DModelCapabilityDelete = 1UL << 6,
    Core3DModelCapabilityDuplicate = 1UL << 7,
    Core3DModelCapabilityMirror = 1UL << 8,
    Core3DModelCapabilityBoolean = 1UL << 9,
    Core3DModelCapabilityChamfer = 1UL << 10,
    Core3DModelCapabilityExtrusion = 1UL << 11,
    Core3DModelCapabilityMaterial = 1UL << 12,
    Core3DModelCapabilityExportOBJ = 1UL << 13,
    Core3DModelCapabilityExportSTL = 1UL << 14,
    Core3DModelCapabilityExportGLB = 1UL << 15,
    Core3DModelCapabilityExportSTEP = 1UL << 16,
    Core3DModelCapabilityLinearArray = 1UL << 17,
    Core3DModelCapabilityShell = 1UL << 18,
    Core3DModelCapabilityRadialArray = 1UL << 19,
};
