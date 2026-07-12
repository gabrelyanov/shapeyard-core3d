//
//  Core3DMaterialController.mm
//  Core3D
//
//  Created by Dmitry Sukhorukov on 18.10.2024.
//

#import <Core3D/Core3DMaterialController.h>
#include "Graphic3d_NameOfMaterial.hxx"
#include "Quantity_Color.hxx"
#include "Quantity_NameOfColor.hxx"
#include "Graphic3d_MaterialAspect.hxx"
#import "NSBundle+Path.h"
#include "ListOfColors.h"
#include <cmath>

@implementation Core3DMaterial {
    
}

-(instancetype) initWithIdentity:(NSUInteger)identity
                        nameKey:(NSString*)nameKey
                    defaultColor:(Core3DColor*)defColor {
    if(self = [super init]) {
        _identity = identity;
        _nameKey = nameKey;
        _defaultColor = defColor;
    }
    return self;
}

@end

@implementation Core3DPBRMaterial

-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                  metallic:(CGFloat)metallic
                                 roughness:(CGFloat)roughness {
    return [self initWithBaseColor:baseColor
                          metallic:metallic
                         roughness:roughness
             supportsScalarEditing:YES
               hasBaseColorTexture:NO
   supportsBaseColorTextureEditing:YES
               hasEmissiveTexture:NO
   supportsEmissiveTextureEditing:YES];
}

-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                  metallic:(CGFloat)metallic
                                 roughness:(CGFloat)roughness
                     supportsScalarEditing:(BOOL)supportsScalarEditing {
    return [self initWithBaseColor:baseColor
                          metallic:metallic
                         roughness:roughness
             supportsScalarEditing:supportsScalarEditing
               hasBaseColorTexture:NO
   supportsBaseColorTextureEditing:supportsScalarEditing
               hasEmissiveTexture:NO
   supportsEmissiveTextureEditing:supportsScalarEditing];
}

-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                  metallic:(CGFloat)metallic
                                 roughness:(CGFloat)roughness
                     supportsScalarEditing:(BOOL)supportsScalarEditing
                       hasBaseColorTexture:(BOOL)hasBaseColorTexture
           supportsBaseColorTextureEditing:(BOOL)supportsBaseColorTextureEditing {
    return [self initWithBaseColor:baseColor
                          metallic:metallic
                         roughness:roughness
             supportsScalarEditing:supportsScalarEditing
               hasBaseColorTexture:hasBaseColorTexture
   supportsBaseColorTextureEditing:supportsBaseColorTextureEditing
               hasEmissiveTexture:NO
   supportsEmissiveTextureEditing:supportsScalarEditing];
}

-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                  metallic:(CGFloat)metallic
                                 roughness:(CGFloat)roughness
                     supportsScalarEditing:(BOOL)supportsScalarEditing
                       hasBaseColorTexture:(BOOL)hasBaseColorTexture
           supportsBaseColorTextureEditing:(BOOL)supportsBaseColorTextureEditing
                       hasEmissiveTexture:(BOOL)hasEmissiveTexture
           supportsEmissiveTextureEditing:(BOOL)supportsEmissiveTextureEditing {
    if(baseColor == nil || !std::isfinite(metallic) || !std::isfinite(roughness)
       || metallic < 0.0 || metallic > 1.0
       || roughness < 0.0 || roughness > 1.0) {
        return nil;
    }
    if(self = [super init]) {
        _baseColor = baseColor;
        _metallic = metallic;
        _roughness = roughness;
        _supportsScalarEditing = supportsScalarEditing;
        _hasBaseColorTexture = hasBaseColorTexture;
        _supportsBaseColorTextureEditing =
            supportsBaseColorTextureEditing;
        _hasEmissiveTexture = hasEmissiveTexture;
        _supportsEmissiveTextureEditing =
            supportsEmissiveTextureEditing;
    }
    return self;
}

-(id)copyWithZone:(NSZone*)zone {
    return [[Core3DPBRMaterial allocWithZone:zone]
        initWithBaseColor:self.baseColor
                 metallic:self.metallic
                roughness:self.roughness
    supportsScalarEditing:self.supportsScalarEditing
      hasBaseColorTexture:self.hasBaseColorTexture
supportsBaseColorTextureEditing:self.supportsBaseColorTextureEditing
      hasEmissiveTexture:self.hasEmissiveTexture
supportsEmissiveTextureEditing:self.supportsEmissiveTextureEditing];
}

-(BOOL)isEqualToPBRMaterial:(Core3DPBRMaterial*)other {
    return other != nil
        && [self.baseColor isEqual:other.baseColor]
        && self.metallic == other.metallic
        && self.roughness == other.roughness
        && self.supportsScalarEditing == other.supportsScalarEditing
        && self.hasBaseColorTexture == other.hasBaseColorTexture
        && self.supportsBaseColorTextureEditing
            == other.supportsBaseColorTextureEditing
        && self.hasEmissiveTexture == other.hasEmissiveTexture
        && self.supportsEmissiveTextureEditing
            == other.supportsEmissiveTextureEditing;
}

-(BOOL)isEqual:(id)object {
    return object == self
        || ([object isKindOfClass:Core3DPBRMaterial.class]
            && [self isEqualToPBRMaterial:(Core3DPBRMaterial*)object]);
}

-(NSUInteger)hash {
    return self.baseColor.hash
        ^ @((double)self.metallic).hash
        ^ @((double)self.roughness).hash
        ^ @(self.supportsScalarEditing).hash
        ^ @(self.hasBaseColorTexture).hash
        ^ @(self.supportsBaseColorTextureEditing).hash
        ^ @(self.hasEmissiveTexture).hash
        ^ @(self.supportsEmissiveTextureEditing).hash;
}

@end

@implementation Core3DColor {
    
}
-(instancetype) initWithIdentity:(NSUInteger)identity
                           color:(UIColor*)color {
    if(self = [super init]) {
        _identity = identity;
        _color = color;
    }
    return self;
}

@end

@implementation Core3DMaterialController {
    
}

-(instancetype) init {
    if(self = [super init]) {
        _selectedMaterials = @[];
        _selectedColors = @[];
        _selectedPBRMaterials = @[];
        [self initColors];
        [self initMaterials];
    }
    return self;
}

-(void) initColors {
    NSMutableArray* a = [NSMutableArray array];
    for(auto c : listOfColors) {
        Quantity_Color qntColor((Quantity_NameOfColor)c);
        UIColor* uiColor = [UIColor colorWithRed:qntColor.Red() green:qntColor.Green() blue:qntColor.Blue() alpha:1.f];
        Core3DColor* color = [[Core3DColor alloc] initWithIdentity:c color:uiColor];
        [a addObject:color];
    }
    _colors = [a copy];
}

-(Core3DColor*) findColorWithName:(NSUInteger)name {
    NSUInteger idx = [_colors indexOfObjectPassingTest:
      ^(Core3DColor* obj, NSUInteger idx, BOOL *stop) {
        return obj.identity == name;
    }];
    
    if(idx == NSNotFound) {
        return nil;
    }
    return _colors[idx];
}

-(void) initMaterials {
    NSMutableArray* a = [NSMutableArray array];
    for(int i = Graphic3d_NameOfMaterial_Brass; i <= Graphic3d_NameOfMaterial_Diamond; ++i) {
        Graphic3d_MaterialAspect materialAspect((Graphic3d_NameOfMaterial)i);
        NSString* key = [NSString stringWithFormat:@"%s", materialAspect.MaterialName()];
        NSString* path = [[NSBundle c3dBundle] pathForResource:[NSString stringWithFormat:@"%@.png", key]  ofType:@""];
        UIImage* thumbImage = [UIImage imageWithContentsOfFile:path];
        Core3DMaterial* m = [[Core3DMaterial alloc] initWithIdentity:i
                                                             nameKey:key
                                                        defaultColor:[self findColorWithName:materialAspect.Color().Name()]];
        m.thumb = thumbImage;
        [a addObject:m];
    }
    _materials = [a copy];
}

-(void) updateSelectionWithMaterial:(Core3DMaterial*)material color:(Core3DColor*)color {
    if(_pass && [_pass respondsToSelector:@selector(updateSelectionWithMaterial:color:)]) {
        [_pass updateSelectionWithMaterial:material color:color];
    }
}

-(void) updateSelectionWithPBRMaterial:(Core3DPBRMaterial*)material {
    if(_pass && [_pass respondsToSelector:@selector(updateSelectionWithPBRMaterial:)]) {
        [_pass updateSelectionWithPBRMaterial:material];
    }
}

-(BOOL)updateSelectionWithBaseColorTextureData:(NSData*)textureData
                                      mediaType:(NSString*)mediaType
                                          error:(NSError* _Nullable * _Nullable)error {
    if(_pass && [_pass respondsToSelector:
            @selector(updateSelectionWithBaseColorTextureData:mediaType:error:)]) {
        const BOOL succeeded = [_pass
            updateSelectionWithBaseColorTextureData:textureData
            mediaType:mediaType
            error:error];
        if (!succeeded && error != nullptr && *error == nil) {
            *error = [NSError errorWithDomain:@"Core3DMaterialControllerError"
                                         code:1
                                     userInfo:@{NSLocalizedDescriptionKey:
                                         @"The base color texture could not be applied."}];
        }
        return succeeded;
    }
    if (error != nullptr) {
        *error = [NSError errorWithDomain:@"Core3DMaterialControllerError"
                                     code:1
                                 userInfo:@{NSLocalizedDescriptionKey:
                                     @"The material editor is unavailable."}];
    }
    return NO;
}

-(BOOL)clearSelectionBaseColorTextureWithError:(NSError* _Nullable * _Nullable)error {
    if(_pass && [_pass respondsToSelector:
            @selector(clearSelectionBaseColorTextureWithError:)]) {
        const BOOL succeeded = [_pass
            clearSelectionBaseColorTextureWithError:error];
        if (!succeeded && error != nullptr && *error == nil) {
            *error = [NSError errorWithDomain:@"Core3DMaterialControllerError"
                                         code:1
                                     userInfo:@{NSLocalizedDescriptionKey:
                                         @"The base color texture could not be removed."}];
        }
        return succeeded;
    }
    if (error != nullptr) {
        *error = [NSError errorWithDomain:@"Core3DMaterialControllerError"
                                     code:1
                                 userInfo:@{NSLocalizedDescriptionKey:
                                     @"The material editor is unavailable."}];
    }
    return NO;
}

-(BOOL)updateSelectionWithEmissiveTextureData:(NSData*)textureData
                                    mediaType:(NSString*)mediaType
                                        error:(NSError* _Nullable * _Nullable)error {
    if(_pass && [_pass respondsToSelector:
            @selector(updateSelectionWithEmissiveTextureData:mediaType:error:)]) {
        const BOOL succeeded = [_pass
            updateSelectionWithEmissiveTextureData:textureData
            mediaType:mediaType
            error:error];
        if (!succeeded && error != nullptr && *error == nil) {
            *error = [NSError errorWithDomain:@"Core3DMaterialControllerError"
                                         code:1
                                     userInfo:@{NSLocalizedDescriptionKey:
                                         @"The emissive texture could not be applied."}];
        }
        return succeeded;
    }
    if (error != nullptr) {
        *error = [NSError errorWithDomain:@"Core3DMaterialControllerError"
                                     code:1
                                 userInfo:@{NSLocalizedDescriptionKey:
                                     @"The material editor is unavailable."}];
    }
    return NO;
}

-(BOOL)clearSelectionEmissiveTextureWithError:(NSError* _Nullable * _Nullable)error {
    if(_pass && [_pass respondsToSelector:
            @selector(clearSelectionEmissiveTextureWithError:)]) {
        const BOOL succeeded = [_pass
            clearSelectionEmissiveTextureWithError:error];
        if (!succeeded && error != nullptr && *error == nil) {
            *error = [NSError errorWithDomain:@"Core3DMaterialControllerError"
                                         code:1
                                     userInfo:@{NSLocalizedDescriptionKey:
                                         @"The emissive texture could not be removed."}];
        }
        return succeeded;
    }
    if (error != nullptr) {
        *error = [NSError errorWithDomain:@"Core3DMaterialControllerError"
                                     code:1
                                 userInfo:@{NSLocalizedDescriptionKey:
                                     @"The material editor is unavailable."}];
    }
    return NO;
}

-(void) didChangeSelectionWithMaterials:(NSArray<Core3DMaterial*>*)materials
                                 colors:(NSArray<Core3DColor*>*) colors {
    _selectedMaterials = materials;
    _selectedColors = colors;
}

-(void) didChangeSelectionWithPBRMaterials:(NSArray<Core3DPBRMaterial*>*)materials {
    _selectedPBRMaterials = [materials copy];
}


@end
