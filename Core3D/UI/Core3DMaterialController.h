//
//  Core3DMaterialController.h
//  Core3D
//
//  Created by Dmitry Sukhorukov on 18.10.2024.
//

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface Core3DColor : NSObject

@property (nonatomic, readonly) NSUInteger identity;
@property (nonatomic, readonly) UIColor* color;

-(instancetype) initWithIdentity:(NSUInteger)identity
                   color:(UIColor*)color;
@end


@interface Core3DMaterial : NSObject

@property (nonatomic, readonly) NSUInteger identity;
@property (nonatomic, readonly) NSString* nameKey;
@property (nonatomic, readonly) Core3DColor* defaultColor;
@property (nonatomic, strong) UIImage* thumb;

-(instancetype) initWithIdentity:(NSUInteger)identity
                        nameKey:(NSString*)nameKey
                   defaultColor:(Core3DColor*)defColor;

@end

//! Editable metallic-roughness material values. `baseColor` is expressed in
//! UIKit's extended-sRGB color space; Core3D converts it to linear RGB before
//! persisting it in the XCAF document and publishing scene snapshots.
@interface Core3DPBRMaterial : NSObject <NSCopying>

@property (nonatomic, strong, readonly) UIColor* baseColor;
@property (nonatomic, assign, readonly) CGFloat metallic;
@property (nonatomic, assign, readonly) CGFloat roughness;
//! True when the effective material owns an embedded base-color texture.
@property (nonatomic, assign, readonly) BOOL hasBaseColorTexture;
//! True when the selected label can safely accept or remove a base-color
//! texture without discarding another texture-map representation.
@property (nonatomic, assign, readonly) BOOL supportsBaseColorTextureEditing;
//! True when the effective material owns an embedded emissive texture.
@property (nonatomic, assign, readonly) BOOL hasEmissiveTexture;
//! True when the selected label can safely accept or remove an emissive
//! texture without discarding another texture-map representation.
@property (nonatomic, assign, readonly) BOOL supportsEmissiveTextureEditing;
//! True for scalar materials and for Shapeyard-owned base-color/emissive
//! textured materials. Imported texture-backed materials remain scalar
//! read-only until the user explicitly authors the relevant texture slot.
@property (nonatomic, assign, readonly) BOOL supportsScalarEditing;

-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                 metallic:(CGFloat)metallic
                                roughness:(CGFloat)roughness;
-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                 metallic:(CGFloat)metallic
                                roughness:(CGFloat)roughness
                    supportsScalarEditing:(BOOL)supportsScalarEditing;
-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                 metallic:(CGFloat)metallic
                                roughness:(CGFloat)roughness
                    supportsScalarEditing:(BOOL)supportsScalarEditing
                      hasBaseColorTexture:(BOOL)hasBaseColorTexture
          supportsBaseColorTextureEditing:(BOOL)supportsBaseColorTextureEditing;
-(nullable instancetype)initWithBaseColor:(UIColor*)baseColor
                                 metallic:(CGFloat)metallic
                                roughness:(CGFloat)roughness
                    supportsScalarEditing:(BOOL)supportsScalarEditing
                      hasBaseColorTexture:(BOOL)hasBaseColorTexture
          supportsBaseColorTextureEditing:(BOOL)supportsBaseColorTextureEditing
                      hasEmissiveTexture:(BOOL)hasEmissiveTexture
          supportsEmissiveTextureEditing:(BOOL)supportsEmissiveTextureEditing;
-(BOOL)isEqualToPBRMaterial:(Core3DPBRMaterial*)other;

@end

@protocol Core3DMaterialControllerDelegate  <NSObject>

@optional
-(void) didSelectionWithMaterials:(NSArray<Core3DMaterial*>*)materials
                        colors:(NSArray<Core3DColor*>*)colors;

@end

@protocol Core3DMaterialControllerCalls  <NSObject>

@optional
-(void) updateSelectionWithMaterial:(Core3DMaterial*)material
                           color:(Core3DColor*)color;
-(void) updateSelectionWithPBRMaterial:(Core3DPBRMaterial*)material;
-(BOOL)updateSelectionWithBaseColorTextureData:(NSData*)textureData
                                      mediaType:(NSString*)mediaType
                                          error:(NSError* _Nullable * _Nullable)error;
-(BOOL)clearSelectionBaseColorTextureWithError:(NSError* _Nullable * _Nullable)error;
-(BOOL)updateSelectionWithEmissiveTextureData:(NSData*)textureData
                                    mediaType:(NSString*)mediaType
                                        error:(NSError* _Nullable * _Nullable)error;
-(BOOL)clearSelectionEmissiveTextureWithError:(NSError* _Nullable * _Nullable)error;

@end

@interface Core3DMaterialController : NSObject

@property (nonatomic, weak) id<Core3DMaterialControllerDelegate> delegate;
@property (nonatomic, weak) id<Core3DMaterialControllerCalls> pass NS_REFINED_FOR_SWIFT;
@property (nonatomic, strong, readonly) NSArray<Core3DMaterial*>* materials;
@property (nonatomic, strong, readonly) NSArray<Core3DColor*>* colors;

@property (nonatomic, strong, readonly) NSArray<Core3DMaterial*>* selectedMaterials;
@property (nonatomic, strong, readonly) NSArray<Core3DColor*>* selectedColors;
@property (nonatomic, strong, readonly) NSArray<Core3DPBRMaterial*>* selectedPBRMaterials;

-(void) updateSelectionWithMaterial:(Core3DMaterial* _Nullable)material color:(Core3DColor* _Nullable)color;
-(void) updateSelectionWithPBRMaterial:(Core3DPBRMaterial*)material;
-(BOOL)updateSelectionWithBaseColorTextureData:(NSData*)textureData
                                      mediaType:(NSString*)mediaType
                                          error:(NSError* _Nullable * _Nullable)error;
-(BOOL)clearSelectionBaseColorTextureWithError:(NSError* _Nullable * _Nullable)error;
-(BOOL)updateSelectionWithEmissiveTextureData:(NSData*)textureData
                                    mediaType:(NSString*)mediaType
                                        error:(NSError* _Nullable * _Nullable)error;
-(BOOL)clearSelectionEmissiveTextureWithError:(NSError* _Nullable * _Nullable)error;
-(void) didChangeSelectionWithMaterials:(NSArray<Core3DMaterial*>*)materials
                                 colors:(NSArray<Core3DColor*>*) colors;
-(void) didChangeSelectionWithPBRMaterials:(NSArray<Core3DPBRMaterial*>*)materials;
-(Core3DColor* _Nullable) findColorWithName:(NSUInteger)name;

@end

NS_ASSUME_NONNULL_END
