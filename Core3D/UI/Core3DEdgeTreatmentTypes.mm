#import "Core3DEdgeTreatmentTypes.h"
#import <objc/runtime.h>
#include "../OCCTKit/RetainedEdgeTreatmentSnapshot.hxx"

static id B1Object(Class type) { return class_createInstance(type, 0); }

@interface Core3DEdgeTreatmentVector3()-(instancetype)initWithX:(double)x y:(double)y z:(double)z;@end
@implementation Core3DEdgeTreatmentVector3
-(instancetype)initWithX:(double)x y:(double)y z:(double)z{if((self=[super init])){_x=x;_y=y;_z=z;}return self;}@end
@implementation Core3DEdgeTreatmentAnchor @end
@implementation Core3DEdgeTreatmentStep @end
@implementation Core3DEdgeTreatmentProfileSource @end
@implementation Core3DEdgeTreatmentEnclosureSource @end
@implementation Core3DEdgeTreatmentSnapshot @end
@implementation Core3DEdgeTreatmentCapture @end
@implementation Core3DEdgeTreatmentTargetCapture @end
@implementation Core3DEdgeTreatmentResult @end

@interface Core3DEdgeTreatmentOperation(){@public std::shared_ptr<core3d::retained_edge_treatment::Work> _nativeWork; BOOL _settled;}@end
@implementation Core3DEdgeTreatmentOperation @end

namespace core3d::edge_treatment_bridge {
NSString *Identifier(const retained_edge_treatment::UUID&value){return [[NSUUID alloc]initWithUUIDBytes:value.data()].UUIDString;}
Core3DEdgeTreatmentVector3 *Vector(const std::array<double,3>&v){return [[Core3DEdgeTreatmentVector3 alloc]initWithX:v[0] y:v[1] z:v[2]];}
Core3DEdgeTreatmentAnchor *Anchor(const retained_edge_treatment::Anchor&value){auto result=(Core3DEdgeTreatmentAnchor *)B1Object(Core3DEdgeTreatmentAnchor.class);[result setValue:[NSData dataWithBytes:value.key.data() length:value.key.size()] forKey:@"key"];[result setValue:@(NSInteger(value.curve)) forKey:@"curve"];[result setValue:Vector(value.pointMM) forKey:@"pointMM"];[result setValue:Vector(value.tangent) forKey:@"tangent"];[result setValue:Vector(value.normalA) forKey:@"normalA"];[result setValue:Vector(value.normalB) forKey:@"normalB"];[result setValue:@(value.circleRadiusMM) forKey:@"circleRadiusMM"];return result;}
} // namespace core3d::edge_treatment_bridge
