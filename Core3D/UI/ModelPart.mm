//
//  ModelPart.m
//  Core3D
//
//  Created by Dmitry Sukhorukov on 24.09.2024.
//

#import "ModelPart.h"

@implementation ModelPart


-(instancetype) init {
    if(self = [super init]) {
        _aligned = NO;
    }
    return self;
}

-(NSString *)description
{
  return [NSString stringWithFormat:@"<ModelPart: %ld, thumb: %@, aligned: %d>",
                 [self identity], [self thumbImage], [self aligned]];
}

@end

