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
             supportsScalarEditing:YES];
}

-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                  metallic:(CGFloat)metallic
                                 roughness:(CGFloat)roughness
                     supportsScalarEditing:(BOOL)supportsScalarEditing {
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
    }
    return self;
}

-(id)copyWithZone:(NSZone*)zone {
    return [[Core3DPBRMaterial allocWithZone:zone]
        initWithBaseColor:self.baseColor
                 metallic:self.metallic
                roughness:self.roughness
    supportsScalarEditing:self.supportsScalarEditing];
}

-(BOOL)isEqualToPBRMaterial:(Core3DPBRMaterial*)other {
    return other != nil
        && [self.baseColor isEqual:other.baseColor]
        && self.metallic == other.metallic
        && self.roughness == other.roughness
        && self.supportsScalarEditing == other.supportsScalarEditing;
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
        ^ @(self.supportsScalarEditing).hash;
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

-(void) didChangeSelectionWithMaterials:(NSArray<Core3DMaterial*>*)materials
                                 colors:(NSArray<Core3DColor*>*) colors {
    _selectedMaterials = materials;
    _selectedColors = colors;
}

-(void) didChangeSelectionWithPBRMaterials:(NSArray<Core3DPBRMaterial*>*)materials {
    _selectedPBRMaterials = [materials copy];
}


@end
