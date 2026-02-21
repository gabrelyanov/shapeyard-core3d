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

@protocol Core3DMaterialControllerDelegate  <NSObject>

@optional
-(void) didSelectionWithMaterials:(NSArray<Core3DMaterial*>*)materials
                        colors:(NSArray<Core3DColor*>*)colors;

@end

@protocol Core3DMaterialControllerCalls  <NSObject>

@optional
-(void) updateSelectionWithMaterial:(Core3DMaterial*)material
                           color:(Core3DColor*)color;

@end

@interface Core3DMaterialController : NSObject

@property (nonatomic, weak) id<Core3DMaterialControllerDelegate> delegate;
@property (nonatomic, weak) id<Core3DMaterialControllerCalls> pass NS_REFINED_FOR_SWIFT;
@property (nonatomic, strong, readonly) NSArray<Core3DMaterial*>* materials;
@property (nonatomic, strong, readonly) NSArray<Core3DColor*>* colors;

@property (nonatomic, strong, readonly) NSArray<Core3DMaterial*>* selectedMaterials;
@property (nonatomic, strong, readonly) NSArray<Core3DColor*>* selectedColors;

-(void) updateSelectionWithMaterial:(Core3DMaterial* _Nullable)material color:(Core3DColor* _Nullable)color;
-(void) didChangeSelectionWithMaterials:(NSArray<Core3DMaterial*>*)materials
                                 colors:(NSArray<Core3DColor*>*) colors;
-(Core3DColor* _Nullable) findColorWithName:(NSUInteger)name;

@end

NS_ASSUME_NONNULL_END
