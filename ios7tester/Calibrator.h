//
//  Calibrator.h
//  ios7tester
//
//  Created by Sam Royston on 5/9/15.
//  Copyright (c) 2015 Sam Fox. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <opencv2/opencv.hpp>
#import <opencv2/highgui/cap_ios.h>

@interface Calibrator : NSObject<CvVideoCameraDelegate>

+(cv::Mat) loadMatFromKey: (NSString*) key;

@end
