//
//  StructureFromMotion.h
//  ios7tester
//
//  Created by Sam Royston on 5/10/15.
//  Copyright (c) 2015 Sam Fox. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <opencv2/opencv.hpp>
#import <opencv2/highgui/cap_ios.h>
#import "Calibrator.h"
#import "OpenGLView.h"
#import "freak.h"

@interface StructureFromMotion : NSObject<CvVideoCameraDelegate>

@property (nonatomic,retain) UIImage *templateImg;
@property (nonatomic,retain) OpenGLView *openGLView;
@property (nonatomic,readwrite) BOOL shouldSave;
@property (nonatomic,readwrite) BOOL shouldRun;

@end
