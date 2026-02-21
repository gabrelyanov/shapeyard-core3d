//
//  Core3DGuidedController.h
//  Core3D
//
//  Created by Dmitry Sukhorukov on 11.09.2024.
//

#import <Core3D/Core3D.h>
#import <Core3D/ModelPart.h>

NS_ASSUME_NONNULL_BEGIN

@protocol GuidedControllerDataSource <NSObject>

@required
-(NSUInteger) partsCount;
-(ModelPart*) partForIndex:(NSInteger)index;

@end

@protocol GuidedControllerDelegate <NSObject>

@optional
-(void) didPartsUpdated;
-(void) didPartAligned:(ModelPart*)part;
-(void) didUpdatePrepareProgress:(NSInteger)current total:(NSUInteger)count;
@end

@interface Core3DGuidedController : Core3DViewController <GuidedControllerDataSource>

@property (nonatomic) NSArray<ModelPart*>* parts;
@property (nonatomic, weak) id<GuidedControllerDelegate> guidedControllerDelegate;

- (void)guidedPlace:(ModelPart*)part;
- (void)loadFromBundle:(NSURL *_Nullable)bundleUrl;
- (void)prepareGuidedParts;
- (void)revertAll;
- (void)didSelectPart:(ModelPart*_Nonnull)part;

@end

NS_ASSUME_NONNULL_END
