// Staged macOS umbrella for the shared Core3D framework.
//
// This surface is intentionally limited to Objective-C value and operation
// APIs whose declarations are free of UIKit. The C++ viewer remains an
// implementation detail until an AppKit session/action facade is extracted.

#import <Foundation/Foundation.h>

FOUNDATION_EXPORT double Core3DVersionNumber;
FOUNDATION_EXPORT const unsigned char Core3DVersionString[];

#import <Core3D/PrimitiveType.h>
#import <Core3D/PrimitiveSelectionType.h>
#import <Core3D/PrimitiveGizmoType.h>
#import <Core3D/ExportType.h>
#import <Core3D/OrthoProjectionType.h>
#import <Core3D/Core3DModelCapability.h>
#import <Core3D/Core3DTransformInspectorSnapshot.h>
#import <Core3D/Core3DSceneSnapshot.h>
#import <Core3D/Core3DNativeExportOperation.h>
#import <Core3D/Core3DNativeImportOperation.h>
#import <Core3D/Core3DMeshContactOperation.h>
#import <Core3D/AssetBundleReaderInterface.h>
#import <Core3D/AssetBundleWriterInterface.h>
#import <Core3D/AssetBundleItem.h>
#import <Core3D/AssetBundle.h>

